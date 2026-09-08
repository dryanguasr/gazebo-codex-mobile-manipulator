import json
import math
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint


STATES = (
    'IDLE', 'FREEZE_BASE', 'OPEN', 'PREGRASP', 'APPROACH', 'CLOSE',
    'VERIFY_GRASP', 'LIFT', 'HOLD', 'TRANSFER', 'LOWER', 'RELEASE',
    'RETREAT', 'DONE', 'RECOVER', 'FAILED', 'CANCELLED',
)
MOTION_STATES = {
    'OPEN': 'home',
    'PREGRASP': 'pregrasp',
    'APPROACH': 'grasp',
    'LIFT': 'lift',
    'TRANSFER': 'transfer',
    'LOWER': 'place',
    'RETREAT': 'retreat',
}

NOMINAL_STATE_SEQUENCE = STATES[:14]


def state_has_timed_out(elapsed_s, timeout_s):
    return elapsed_s > timeout_s


def wall_watchdog_expired(elapsed_wall_s, timeout_s):
    return elapsed_wall_s > timeout_s


def may_retry_goal_rejection(attempt, maximum_retries):
    return attempt <= maximum_retries


def recovery_lowering_pose(attached, transfer_started):
    if not attached:
        return None
    return 'place' if transfer_started else 'grasp'


def recovery_terminal_state(cancel_requested):
    return 'CANCELLED' if cancel_requested else 'FAILED'


class PickPlaceSupervisor(Node):
    """Single owner of m1-m6 and deterministic Level-A state machine."""

    def __init__(self):
        super().__init__('pick_place_supervisor')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('grasp_mode', 'attach_conditioned')
        self.declare_parameter(
            'joint_names',
            [f'poppy_m{i}_joint' for i in range(1, 7)],
        )
        defaults = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 1.2],
            'pregrasp_clearance_1': [-1.0, 0.0, 0.0, 0.0, 0.0, 1.2],
            'pregrasp_clearance_2': [-1.0, 1.3835, -0.822123, 0.045428, -0.742069, 1.2],
            'pregrasp': [-0.0208, 1.3835, -0.822123, 0.045428, -0.742069, 1.2],
            'grasp': [0.026497, 1.45, -0.580578, 0.000013, -0.753151, 1.2],
            'close': [0.026497, 1.45, -0.580578, 0.000013, -0.753151, 0.0],
            'lift': [-0.0208, 1.3835, -0.822123, 0.045428, -0.742069, 0.60],
            'transfer': [0.362891, 1.394351, -0.822774, 0.065445, -0.744443, 0.60],
            'place': [0.422991, 1.45, -0.580630, 0.000013, -0.753150, 0.65],
            'release_open': [0.422991, 1.45, -0.580630, 0.000013, -0.753150, 1.2],
            'retreat': [0.362891, 1.394351, -0.822774, 0.065445, -0.744443, 1.2],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name, value in (
            ('motion_duration_s', 2.5),
            ('joint_tolerance_rad', 0.035),
            ('state_timeout_s', 8.0),
            ('hold_duration_s', 3.2),
            ('release_stability_s', 2.2),
            ('base_speed_limit_mps', 0.01),
            ('base_drift_limit_m', 0.01),
            ('wall_watchdog_s', 3.0),
            ('goal_rejection_retry_delay_s', 0.25),
        ):
            self.declare_parameter(name, value)
        self.declare_parameter('goal_rejection_retries', 1)
        self.declare_parameter(
            'action_name', '/arm_controller/follow_joint_trajectory'
        )

        self.state = 'IDLE'
        self.state_started_ns = self.get_clock().now().nanoseconds
        self.state_started_wall = time.monotonic()
        self.last_sim_ns = self.state_started_ns
        self.last_sim_progress_wall = time.monotonic()
        self.joints = {}
        self.odom = None
        self.base_origin = None
        self.attached = False
        self.physical_grasp_verified = False
        self.motion_started = False
        self.motion_done = False
        self.motion_error = None
        self.motion_attempt = 0
        self.motion_retry_after_wall = 0.0
        self.goal_handle = None
        self.close_commanded = False
        self.cancel_requested = False
        self.failure_reason = ''
        self.recovery_stage = 0
        self.transfer_started = False
        self.recovery_release_logged = False
        self.terminal_wall = None

        self.last_feedback_emit_wall = 0.0
        self.action = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter('action_name').value),
        )
        self.status_pub = self.create_publisher(
            String, '/pick_place/status', 50
        )
        self.gate_pub = self.create_publisher(
            String, '/pick_place/gate_command', 20
        )
        self.zero_pub = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 20
        )
        self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 20
        )
        self.create_subscription(
            Odometry, '/base_controller/odom', self.odom_callback, 20
        )
        self.create_subscription(
            String, '/pick_object/joint_state', self.attach_callback, 20
        )
        self.create_subscription(
            Bool, '/pick_place/physical_grasp_verified', self.physical_callback, 10
        )
        self.create_subscription(
            Bool, '/pick_place/cancel', self.cancel_callback, 10
        )
        self.create_timer(
            0.02,
            self.tick,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self.emit(
            'supervisor_started',
            action_endpoint=str(self.get_parameter('action_name').value),
            action_type='control_msgs/action/FollowJointTrajectory',
            states=list(STATES),
            control_source='predefined_joint_trajectories',
            ground_truth_used_for_control=False,
        )

    def sim_ns(self):
        return self.get_clock().now().nanoseconds

    def elapsed_sim(self):
        return max(0.0, (self.sim_ns() - self.state_started_ns) / 1e9)

    def emit(self, event, **fields):
        msg = String()
        msg.data = json.dumps(
            {
                'source': 'supervisor',
                'event': event,
                'state': self.state,
                'sim_time_s': self.sim_ns() / 1e9,
                'wall_time_s': time.time(),
                'attached': self.attached,
                **fields,
            },
            sort_keys=True,
        )
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def joint_callback(self, message):
        for index, name in enumerate(message.name):
            if index < len(message.position):
                self.joints[name] = float(message.position[index])

    def odom_callback(self, message):
        self.odom = message
        if self.base_origin is None:
            p = message.pose.pose.position
            self.base_origin = (float(p.x), float(p.y))

    def attach_callback(self, message):
        self.attached = message.data == 'attached'

    def physical_callback(self, message):
        self.physical_grasp_verified = bool(message.data)

    def cancel_callback(self, message):
        if message.data and self.state not in ('DONE', 'FAILED', 'CANCELLED'):
            self.cancel_requested = True

    def base_metrics(self):
        if self.odom is None or self.base_origin is None:
            return math.inf, math.inf
        p = self.odom.pose.pose.position
        drift = math.hypot(p.x - self.base_origin[0], p.y - self.base_origin[1])
        t = self.odom.twist.twist
        speed = math.sqrt(t.linear.x ** 2 + t.linear.y ** 2 + t.angular.z ** 2)
        return drift, speed

    def gate_command(self, state=None):
        msg = String()
        msg.data = json.dumps(
            {
                'state': state or self.state,
                'close_commanded': self.close_commanded,
                'sim_time_s': self.sim_ns() / 1e9,
            },
            sort_keys=True,
        )
        self.gate_pub.publish(msg)

    def transition(self, state, reason='condition_met'):
        old = self.state
        self.state = state
        if state == 'OPEN' and self.odom is not None:
            p = self.odom.pose.pose.position
            self.base_origin = (float(p.x), float(p.y))
        self.state_started_ns = self.sim_ns()
        self.state_started_wall = time.monotonic()
        self.motion_started = False
        self.motion_done = False
        self.motion_error = None
        self.motion_attempt = 0
        self.motion_retry_after_wall = 0.0
        self.goal_handle = None
        self.emit('transition', from_state=old, to_state=state, reason=reason)

    def fail(self, reason, cancelled=False):
        if self.state in ('RECOVER', 'FAILED', 'CANCELLED', 'DONE'):
            return
        self.failure_reason = reason
        self.cancel_requested = cancelled
        self.transition('RECOVER', reason=reason)
        self.recovery_stage = 0
        self.recovery_release_logged = False

    def positions(self, name):
        values = [
            float(x) for x in self.get_parameter(name).value
        ]
        physical_mode = (
            str(self.get_parameter('grasp_mode').value) == 'physical_contact'
        )
        if name in ('lift', 'transfer', 'place') and (
            self.attached or physical_mode
        ):
            current_m6 = self.joints.get('poppy_m6_joint', values[5])
            values[5] = current_m6
        return values

    def start_motion(self, pose_name, duration_s=None):
        if self.motion_started:
            return
        if time.monotonic() < self.motion_retry_after_wall:
            return
        if not self.action.server_is_ready():
            if not self.action.wait_for_server(timeout_sec=0.05):
                return
        names = list(self.get_parameter('joint_names').value)
        pose_names = [pose_name]
        if pose_name == 'pregrasp':
            pose_names = [
                'pregrasp_clearance_1',
                'pregrasp_clearance_2',
                'pregrasp',
            ]
        waypoint_positions = [self.positions(name) for name in pose_names]
        if (
            len(names) != 6
            or any(len(positions) != 6 for positions in waypoint_positions)
        ):
            self.fail('invalid_pose_configuration')
            return
        seconds = float(
            duration_s
            if duration_s is not None
            else self.get_parameter('motion_duration_s').value
        )
        timestamps = [seconds]
        if pose_name == 'pregrasp':
            timestamps = [1.5, 3.0, 5.5]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for positions, timestamp in zip(waypoint_positions, timestamps):
            point = JointTrajectoryPoint()
            point.positions = positions
            whole = int(timestamp)
            point.time_from_start = Duration(
                sec=whole, nanosec=int((timestamp - whole) * 1e9)
            )
            goal.trajectory.points.append(point)
        self.motion_started = True
        self.motion_done = False
        self.motion_attempt += 1
        self.emit(
            'trajectory_sent',
            pose=pose_name,
            waypoints=waypoint_positions,
            duration_s=timestamps[-1],
            attempt=self.motion_attempt,
        )
        future = self.action.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(self.goal_response)

    def feedback(self, feedback):
        desired = feedback.feedback.desired.positions
        actual = feedback.feedback.actual.positions
        if desired and actual and len(desired) == len(actual):
            error = max(abs(a - d) for a, d in zip(actual, desired))
            now = time.monotonic()
            if now - self.last_feedback_emit_wall >= 0.5:
                self.last_feedback_emit_wall = now
                self.emit('trajectory_feedback', max_joint_error_rad=error)

    def goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self.motion_error = f'goal_exception:{error}'
            return
        if not handle.accepted:
            maximum_retries = int(
                self.get_parameter('goal_rejection_retries').value
            )
            if may_retry_goal_rejection(
                self.motion_attempt, maximum_retries
            ):
                delay = float(
                    self.get_parameter(
                        'goal_rejection_retry_delay_s'
                    ).value
                )
                self.emit(
                    'trajectory_goal_rejected_retry',
                    attempt=self.motion_attempt,
                    maximum_retries=maximum_retries,
                    retry_delay_s=delay,
                )
                self.motion_started = False
                self.motion_retry_after_wall = time.monotonic() + delay
            else:
                self.motion_error = 'goal_rejected'
            return
        self.goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(self.goal_result)

    def goal_result(self, future):
        try:
            wrapped = future.result()
            code = int(wrapped.result.error_code)
            if code != 0:
                self.motion_error = f'follow_joint_trajectory_error_{code}'
            else:
                self.motion_done = True
                self.emit(
                    'trajectory_result',
                    observed_positions=self.joints,
                )
        except Exception as error:
            self.motion_error = f'result_exception:{error}'

    def joints_at(self, pose_name, ignore_m6=False):
        names = list(self.get_parameter('joint_names').value)
        desired = self.positions(pose_name)
        if any(name not in self.joints for name in names):
            return False
        errors = [
            abs(self.joints[name] - target)
            for name, target in zip(names, desired)
            if not (ignore_m6 and name == 'poppy_m6_joint')
        ]
        return max(errors, default=math.inf) <= float(
            self.get_parameter('joint_tolerance_rad').value
        )

    def normal_tick(self):
        if self.state == 'IDLE':
            if (
                bool(self.get_parameter('auto_start').value)
                and self.elapsed_sim() >= 0.5
            ):
                self.transition('FREEZE_BASE')
            return
        if self.state == 'FREEZE_BASE':
            if (
                self.odom is not None
                and len(self.joints) >= 6
                and self.action.server_is_ready()
                and self.elapsed_sim() >= 1.5
            ):
                self.transition('OPEN')
            return
        if self.state in MOTION_STATES:
            pose = MOTION_STATES[self.state]
            self.start_motion(pose)
            if self.motion_error:
                self.fail(self.motion_error)
                return
            arrived = self.motion_done and self.joints_at(
                pose, ignore_m6=self.state in ('LIFT', 'TRANSFER', 'LOWER')
            )
            if self.state == 'OPEN' and self.motion_done:
                arrived = abs(
                    self.joints.get('poppy_m6_joint', math.inf) - 1.20
                ) <= 0.03
            if arrived:
                next_state = {
                    'OPEN': 'PREGRASP',
                    'PREGRASP': 'APPROACH',
                    'APPROACH': 'CLOSE',
                    'LIFT': 'HOLD',
                    'TRANSFER': 'LOWER',
                    'LOWER': 'RELEASE',
                    'RETREAT': 'DONE',
                }[self.state]
                if self.state == 'TRANSFER':
                    self.transfer_started = True
                self.transition(next_state)
            return
        if self.state == 'CLOSE':
            self.close_commanded = True
            self.start_motion('close', duration_s=3.0)
            m6 = self.joints.get('poppy_m6_joint', math.inf)
            if m6 <= 0.72 and self.elapsed_sim() >= 0.2:
                self.transition('VERIFY_GRASP')
            elif self.motion_error:
                self.fail(self.motion_error)
            return
        if self.state == 'VERIFY_GRASP':
            self.close_commanded = True
            physical_mode = (
                str(self.get_parameter('grasp_mode').value) == 'physical_contact'
            )
            verified = self.physical_grasp_verified if physical_mode else self.attached
            if verified:
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.transition('LIFT')
            return
        if self.state == 'HOLD':
            physical_mode = (
                str(self.get_parameter('grasp_mode').value) == 'physical_contact'
            )
            if not physical_mode and not self.attached:
                self.fail('object_lost_during_hold')
            elif self.elapsed_sim() >= float(
                self.get_parameter('hold_duration_s').value
            ):
                self.transition('TRANSFER')
            return
        if self.state == 'RELEASE':
            self.gate_command('RELEASE')
            if not self.attached:
                self.close_commanded = False
                if not self.motion_started:
                    self.start_motion('release_open')
                stable_after_open_s = float(
                    self.get_parameter('motion_duration_s').value
                ) + float(self.get_parameter('release_stability_s').value)
                if self.motion_done and self.elapsed_sim() >= stable_after_open_s:
                    self.transition('RETREAT')
            return

    def recovery_tick(self):
        # Attached failures are lowered before the only authorized release.
        if self.attached:
            if self.recovery_stage == 0:
                pose = recovery_lowering_pose(self.attached, self.transfer_started)
                self.start_motion(pose)
                if self.motion_error:
                    self.emit(
                        'recovery_release_withheld',
                        reason=self.motion_error,
                        lowering_pose=pose,
                    )
                    terminal = recovery_terminal_state(self.cancel_requested)
                    self.transition(terminal, reason=self.failure_reason)
                elif self.motion_done:
                    self.recovery_stage = 1
                    self.motion_started = False
                    self.motion_done = False
                    if not self.recovery_release_logged:
                        self.recovery_release_logged = True
                        self.emit(
                            'recovery_release_authorized',
                            reason=self.failure_reason,
                            lowering_pose=pose,
                            normal_release=False,
                        )
            elif self.recovery_stage == 1:
                self.gate_command('RELEASE')
                if not self.attached:
                    self.recovery_stage = 2
                    self.motion_started = False
            return
        if self.recovery_stage < 2:
            self.recovery_stage = 2
            self.motion_started = False
        if self.recovery_stage == 2:
            self.start_motion('home')
            if self.motion_done or self.motion_error or self.elapsed_sim() > 6.0:
                terminal = recovery_terminal_state(self.cancel_requested)
                self.transition(terminal, reason=self.failure_reason)

    def tick(self):
        sim_ns = self.sim_ns()
        if sim_ns > self.last_sim_ns:
            self.last_sim_ns = sim_ns
            self.last_sim_progress_wall = time.monotonic()

        zero = TwistStamped()
        zero.header.stamp = self.get_clock().now().to_msg()
        zero.header.frame_id = 'base_footprint'
        self.zero_pub.publish(zero)
        self.gate_command()

        if self.state in ('DONE', 'FAILED', 'CANCELLED'):
            if self.terminal_wall is None:
                self.terminal_wall = time.monotonic()
                self.emit('terminal', result=self.state, reason=self.failure_reason)
            elif time.monotonic() - self.terminal_wall > 1.5:
                rclpy.shutdown()
            return

        watchdog_elapsed = time.monotonic() - self.last_sim_progress_wall
        watchdog_limit = float(
            self.get_parameter('wall_watchdog_s').value
        )
        if self.state != 'IDLE' and wall_watchdog_expired(
            watchdog_elapsed, watchdog_limit
        ):
            if self.state == 'RECOVER':
                terminal = recovery_terminal_state(self.cancel_requested)
                self.transition(
                    terminal,
                    reason='sim_clock_watchdog_during_recovery',
                )
            else:
                self.fail('sim_clock_watchdog')
            return

        if self.cancel_requested and self.state != 'RECOVER':
            self.fail('explicit_cancel', cancelled=True)
        drift, speed = self.base_metrics()
        if self.state not in ('IDLE', 'FREEZE_BASE', 'RECOVER'):
            if drift > float(self.get_parameter('base_drift_limit_m').value):
                self.fail('base_drift_limit')
            elif speed > float(self.get_parameter('base_speed_limit_mps').value):
                self.fail('base_speed_limit')

        if self.state == 'RECOVER':
            self.recovery_tick()
            return
        if self.state != 'IDLE' and (
            state_has_timed_out(
                self.elapsed_sim(),
                float(self.get_parameter('state_timeout_s').value),
            )
        ):
            self.fail(f'state_timeout:{self.state}')
            return
        self.normal_tick()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
