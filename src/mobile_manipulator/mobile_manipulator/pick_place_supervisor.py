import json
import math
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import TransformStamped, TwistStamped
from mobile_manipulator.mobile_transport import (
    model_planar_pose, PlanarRoute, SmoothRoute, TRANSPORT_STATES, world_to_odom,
    yaw_from_quaternion,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
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
    'FOLD': 'transport_pose',
    'TRANSFER': 'transfer',
    'LOWER': 'place',
    'RETREAT': 'retreat',
}

NOMINAL_STATE_SEQUENCE = STATES[:14]
MOBILE_STATE_SEQUENCE = (
    *NOMINAL_STATE_SEQUENCE[:9], 'FOLD', *TRANSPORT_STATES,
    *NOMINAL_STATE_SEQUENCE[9:],
)


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
        self.declare_parameter('mobile_transport', False)
        self.declare_parameter('route', [
            0.838, 0.0, 0.0, 0.838, 0.0, 1.570796327,
            0.838, 0.35, 1.570796327, 0.838, 0.738, 1.570796327,
        ])
        self.declare_parameter('navigation_timeout_s', 180.0)
        self.declare_parameter('smooth_transport', False)
        self.mobile = bool(self.get_parameter('mobile_transport').value)
        self.world_base = None
        self.world_base_ns = 0
        self.odom_rx_ns = 0
        self.nav_tick_ns = 0
        self.dock_stopped_since = None
        values = list(self.get_parameter('route').value)
        self.route = (SmoothRoute(values[-3:])
                      if self.get_parameter('smooth_transport').value
                      else PlanarRoute([values[i:i + 3] for i in range(0, len(values), 3)]))
        self.localization_tf = TransformBroadcaster(self)
        self.create_subscription(
            TFMessage, '/model/mobile_manipulator/pose', self.base_pose_callback, 20
        )
        self.declare_parameter('grasp_mode', 'attach_conditioned')
        if self.mobile and self.get_parameter('grasp_mode').value != 'attach_conditioned':
            raise ValueError('Mobile exercise requires contact-gated temporary attachment')
        self.declare_parameter(
            'joint_names',
            [f'poppy_m{i}_joint' for i in range(1, 7)],
        )
        defaults = {
            'transport_pose': [-0.05806242, 0.15, -1.05, 0.0, 0.9, 0.0],
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 1.2],
            'pregrasp_clearance_1': [-1.0, 0.0, 0.0, 0.0, 0.0, 1.2],
            'pregrasp_clearance_2': [-1.0, 0.86082245, -0.30659458, 0.0, -0.55422787, 1.2],
            'pregrasp': [-0.05806242, 0.86082245, -0.30659458, 0.0, -0.55422787, 1.2],
            'grasp': [-0.05548826, 1.42673829, -0.3318062, 0.0, -1.09493209, 1.2],
            'close': [-0.05548826, 1.42673829, -0.3318062, 0.0, -1.09493209, 0.0],
            'lift': [-0.05806242, 0.86082245, -0.30659458, 0.0, -0.55422787, 0.0],
            'transfer': [0.35650417, 0.88430762, -0.34516417, 0.0, -0.53914345, 0.0],
            'place': [0.34102488, 1.39662108, -0.3160789, 0.0, -1.08054218, 0.0],
            'release_open': [0.34102488, 1.39662108, -0.3160789, 0.0, -1.08054218, 1.2],
            'retreat': [0.35650417, 0.88430762, -0.34516417, 0.0, -0.53914345, 1.2],
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
        self.physical_feedback_ns = 0
        self.grasp_lost_since_ns = None
        self.motion_generation = 0
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
        for topic in ('/pick_support/contacts', '/place_support/contacts'):
            self.create_subscription(Contacts, topic, self.support_contact_callback, 20)
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
            states=list(MOBILE_STATE_SEQUENCE if self.mobile else STATES),
            control_source='joint_trajectories+localized_base_route' if self.mobile
            else 'predefined_joint_trajectories',
            ground_truth_used_for_control=self.mobile,
            base_localization='Gazebo model pose' if self.mobile else 'wheel odometry',
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

    def support_contact_callback(self, message):
        if not self.mobile:
            return
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any('mobile_manipulator::' in name for name in names):
                self.emit('robot_support_collision', collisions=list(names))
                self.fail('robot_contacted_station')
                return

    def base_pose_callback(self, message):
        pose = model_planar_pose(message)
        if pose is None:
            return
        self.world_base = pose
        self.world_base_ns = self.sim_ns()
        if self.mobile and self.odom is not None:
            p = self.odom.pose.pose.position
            q = self.odom.pose.pose.orientation
            correction = world_to_odom(pose, (p.x, p.y, yaw_from_quaternion(q)))
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = 'world'
            tf.child_frame_id = 'odom'
            tf.transform.translation.x = correction[0]
            tf.transform.translation.y = correction[1]
            tf.transform.rotation.z = math.sin(correction[2] / 2.0)
            tf.transform.rotation.w = math.cos(correction[2] / 2.0)
            self.localization_tf.sendTransform(tf)

    def odom_callback(self, message):
        self.odom_rx_ns = self.sim_ns()
        self.odom = message
        if self.base_origin is None:
            p = message.pose.pose.position
            self.base_origin = (float(p.x), float(p.y))

    def attach_callback(self, message):
        self.attached = message.data == 'attached'

    def physical_callback(self, message):
        self.physical_grasp_verified = bool(message.data)
        self.physical_feedback_ns = self.sim_ns()

    def cancel_callback(self, message):
        if message.data and self.state not in ('DONE', 'FAILED', 'CANCELLED'):
            self.cancel_requested = True

    def base_metrics(self):
        if self.odom is None or self.base_origin is None:
            return math.inf, math.inf
        p = self.odom.pose.pose.position
        xy = self.world_base[:2] if self.mobile and self.world_base else (p.x, p.y)
        drift = math.dist(xy, self.base_origin)
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
        if (state == 'OPEN' or (self.mobile and state == 'TRANSFER')) and self.odom is not None:
            p = self.odom.pose.pose.position
            self.base_origin = (
                self.world_base[:2] if self.mobile and self.world_base
                else (float(p.x), float(p.y))
            )
        self.state_started_ns = self.sim_ns()
        self.state_started_wall = time.monotonic()
        self.motion_started = False
        self.motion_done = False
        self.motion_error = None
        self.motion_attempt = 0
        self.motion_retry_after_wall = 0.0
        self.emit('transition', from_state=old, to_state=state, reason=reason)

    def fail(self, reason, cancelled=False):
        if self.state in ('RECOVER', 'FAILED', 'CANCELLED', 'DONE'):
            return
        self.publish_base()
        self.motion_generation += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.failure_reason = reason
        self.cancel_requested = cancelled
        self.transition('RECOVER', reason=reason)
        self.recovery_stage = 0
        self.recovery_release_logged = False

    def positions(self, name):
        if name == 'recovery_hold':
            names = self.get_parameter('joint_names').value
            values = [self.joints[joint] for joint in names]
            values[5] = float(self.get_parameter('close').value[5])
            return values
        values = [
            float(x) for x in self.get_parameter(name).value
        ]
        if name in ('lift', 'transport_pose', 'transfer', 'place'):
            # Keep closing against the object; replaying the measured angle
            # removes the preload needed for frictional retention.
            values[5] = float(self.get_parameter('close').value[5])
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
            timestamps = [t * seconds / 2.5 for t in (1.5, 3.0, 5.5)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        for positions, timestamp in zip(waypoint_positions, timestamps):
            point = JointTrajectoryPoint()
            point.positions = positions
            if self.mobile:
                # Quintic interpolation gives zero velocity/acceleration at rest.
                point.velocities = [0.0] * len(names)
                point.accelerations = [0.0] * len(names)
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
        self.motion_generation += 1
        generation = self.motion_generation
        future = self.action.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(
            lambda done: self.goal_response(done, generation)
        )

    def feedback(self, feedback):
        desired = feedback.feedback.desired.positions
        actual = feedback.feedback.actual.positions
        if desired and actual and len(desired) == len(actual):
            error = max(abs(a - d) for a, d in zip(actual, desired))
            now = time.monotonic()
            if now - self.last_feedback_emit_wall >= 0.5:
                self.last_feedback_emit_wall = now
                self.emit('trajectory_feedback', max_joint_error_rad=error)

    def goal_response(self, future, generation):
        try:
            handle = future.result()
            if generation != self.motion_generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
        except Exception as error:
            if generation != self.motion_generation:
                return
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
        result.add_done_callback(
            lambda done: self.goal_result(done, generation)
        )

    def goal_result(self, future, generation):
        if generation != self.motion_generation:
            return
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
                and (not self.mobile or self.world_base is not None)
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
                pose, ignore_m6=self.state in ('LIFT', 'FOLD', 'TRANSFER', 'LOWER')
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
                    'FOLD': 'NAVIGATE',
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
            if m6 <= 0.20 and self.elapsed_sim() >= 0.2:
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
                # The next goal supersedes CLOSE and keeps its closing target.
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
                self.transition('FOLD' if self.mobile else 'TRANSFER')
            return
        if self.state == 'NAVIGATE':
            now = self.sim_ns()
            if (
                self.world_base is None or (now - self.world_base_ns) / 1e9 > 0.3
                or (now - self.odom_rx_ns) / 1e9 > 0.3
            ):
                self.publish_base()
                self.fail('base_localization_or_odometry_stale')
                return
            dt = (now - self.nav_tick_ns) / 1e9 if self.nav_tick_ns else 0.0
            self.nav_tick_ns = now
            previous_index = self.route.index
            linear, angular, done = self.route.step(self.world_base, dt)
            self.publish_base(linear, angular)
            if self.route.index != previous_index:
                self.emit('base_waypoint_reached', waypoint=previous_index,
                          actual_world_pose=self.world_base,
                          identified_turn_pivot_m=self.route.pivot_offset)
            if done:
                self.publish_base()
                self.transition('DOCK_BASE')
            return
        if self.state == 'DOCK_BASE':
            _, speed = self.base_metrics()
            if speed <= 0.005:
                if self.dock_stopped_since is None:
                    self.dock_stopped_since = self.sim_ns()
                elif (self.sim_ns() - self.dock_stopped_since) / 1e9 >= 1.0:
                    self.transition('TRANSFER')
            else:
                self.dock_stopped_since = None
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
        if self.mobile or str(self.get_parameter('grasp_mode').value) == 'physical_contact':
            # Do not open or sweep home with a possibly retained/dropped object.
            self.start_motion('recovery_hold', duration_s=0.25)
            if self.motion_done or self.motion_error or self.elapsed_sim() > 2.0:
                self.transition(
                    recovery_terminal_state(self.cancel_requested),
                    reason=self.failure_reason,
                )
            return
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

    def publish_base(self, linear=0.0, angular=0.0):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_footprint'
        command.twist.linear.x = linear
        command.twist.angular.z = angular
        self.zero_pub.publish(command)

    def tick(self):
        sim_ns = self.sim_ns()
        if sim_ns > self.last_sim_ns:
            self.last_sim_ns = sim_ns
            self.last_sim_progress_wall = time.monotonic()

        if self.state != 'NAVIGATE':
            self.publish_base()
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
        if self.state not in ('IDLE', 'FREEZE_BASE', 'RECOVER', *TRANSPORT_STATES):
            if drift > float(self.get_parameter('base_drift_limit_m').value):
                self.fail('base_drift_limit')
            elif speed > float(self.get_parameter('base_speed_limit_mps').value):
                self.fail('base_speed_limit')

        if (self.mobile
                and self.state in ('FOLD', 'TRANSFER', 'LOWER', *TRANSPORT_STATES)
                and not self.attached):
            self.fail('temporary_joint_lost_during_transport')

        if (
            str(self.get_parameter('grasp_mode').value) == 'physical_contact'
            and self.state in ('LIFT', 'HOLD', 'FOLD', 'TRANSFER', 'LOWER', *TRANSPORT_STATES)
        ):
            fresh = (sim_ns - self.physical_feedback_ns) / 1e9 <= 0.25
            if fresh and self.physical_grasp_verified:
                self.grasp_lost_since_ns = None
            elif self.grasp_lost_since_ns is None:
                self.grasp_lost_since_ns = sim_ns
            elif (sim_ns - self.grasp_lost_since_ns) / 1e9 > 0.25:
                self.fail('physical_grasp_lost_or_stale')

        if self.state == 'RECOVER':
            self.recovery_tick()
            return
        if self.state != 'IDLE' and (
            state_has_timed_out(
                self.elapsed_sim(),
                float(self.get_parameter(
                    'navigation_timeout_s' if self.state == 'NAVIGATE'
                    else 'state_timeout_s'
                ).value),
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
