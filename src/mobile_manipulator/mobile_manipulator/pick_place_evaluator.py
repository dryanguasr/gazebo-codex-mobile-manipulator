import csv
import json
import math
from pathlib import Path
import time

from mobile_manipulator.pick_place_attach_gate import extract_single_model_pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None

    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def update_spatial_stability(
    anchor,
    stable_since,
    best_duration,
    sim_time,
    object_pose,
    target_xy,
    placement_tolerance_m,
    motion_tolerance_m,
    attached,
):
    """Track continuous post-release stability inside a spatial envelope."""
    if (
        attached
        or object_pose is None
        or math.dist(object_pose[:2], target_xy) > placement_tolerance_m
    ):
        return None, None, best_duration
    if (
        anchor is None
        or math.dist(anchor, object_pose) > motion_tolerance_m
    ):
        anchor = object_pose
        stable_since = sim_time
    if stable_since is None:
        stable_since = sim_time
    return (
        anchor,
        stable_since,
        max(best_duration, sim_time - stable_since),
    )


class PickPlaceEvaluator(Node):
    """Independent evaluator using actual Gazebo Pose_V state."""

    def __init__(self):
        super().__init__('pick_place_evaluator')
        defaults = (
            ('object_name', 'pick_object'),
            ('tool_frame', 'poppy_tool_frame'),
            ('grasp_frame', 'poppy_grasp_frame'),
            ('place_x_m', 0.044),
            ('place_y_m', -0.213),
            ('placement_tolerance_m', 0.030),
            ('minimum_lift_m', 0.050),
            ('object_height_m', 0.045),
            ('pick_support_top_z_m', 0.140),
            ('minimum_support_clearance_m', 0.005),
            ('minimum_hold_s', 3.0),
            ('minimum_stable_s', 2.0),
            ('base_drift_limit_m', 0.010),
            ('stability_motion_tolerance_m', 0.0005),
            ('output_dir', '/tmp/pick_place_a1'),
            ('run_id', 'run'),
            ('source_sha', 'unknown'),
            ('source_dirty', False),
            ('world', 'pick_and_place.sdf'),
            ('grasp_mode', 'attach_conditioned'),
            ('seed', 1),
        )
        for name, value in defaults:
            self.declare_parameter(name, value)
        self.output = Path(str(self.get_parameter('output_dir').value))
        self.output.mkdir(parents=True, exist_ok=True)
        self.events_path = self.output / 'events.jsonl'
        self.samples_path = self.output / 'samples.csv'
        self.events_handle = self.events_path.open('w', encoding='utf-8')
        self.samples_handle = self.samples_path.open(
            'w', encoding='utf-8', newline=''
        )
        self.sample_fields = [
            'sim_time_s', 'wall_time_s', 'state', 'object_x_m',
            'object_y_m', 'object_z_m', 'attached', 'base_x_m',
            'base_y_m', 'tool_x_m', 'tool_y_m', 'tool_z_m',
            'grasp_x_m', 'grasp_y_m', 'grasp_z_m',
        ]
        self.writer = csv.DictWriter(
            self.samples_handle, fieldnames=self.sample_fields
        )
        self.writer.writeheader()

        self.state = 'IDLE'
        self.started_wall_time_s = time.time()
        self.started_sim_time_s = (
            self.get_clock().now().nanoseconds / 1e9
        )
        self.object_pose = None
        self.object_pose_rx_wall = 0.0
        self.initial_pose = None
        self.grasp_object_pose = None
        self.lift_object_pose = None
        self.max_z = -math.inf
        self.finalized = False
        self.attached = False
        self.attach_count = 0
        self.detach_count = 0
        self.attach_gate_snapshots = []
        self.transitions = []
        self.forbidden_contacts = []
        self.base_origin = None
        self.forbidden_contact_keys = set()
        self.base_pose = None
        self.tool_pose = None
        self.grasp_pose = None
        self.initial_tool_pose = None
        self.max_base_drift = 0.0
        self.hold_started = None
        self.hold_duration = 0.0
        self.detached_at = None
        self.stable_since = None
        self.stable_duration = 0.0
        self.stability_anchor = None
        self.pregrasp_position_error = None
        self.pregrasp_angle_error = None
        self.max_joint_tracking_error = 0.0
        self.trajectory_goal_count = 0
        self.trajectory_retry_count = 0
        self.terminal_payload = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.result_pub = self.create_publisher(
            Bool, '/pick_place/evaluation_complete', 10
        )
        self.create_subscription(
            String, '/pick_place/status', self.status_callback, 50
        )
        self.create_subscription(
            String, '/pick_place/gate_event', self.gate_callback, 50
        )
        self.create_subscription(
            TFMessage,
            '/model/pick_object/pose',
            self.pose_callback,
            50,
        )
        self.create_subscription(
            Contacts, '/pick_object/contacts', self.contact_callback, 50
        )
        self.create_subscription(
            String, '/pick_object/joint_state', self.attach_callback, 20
        )
        self.create_subscription(
            Odometry, '/base_controller/odom', self.odom_callback, 20
        )
        self.create_timer(
            0.02,
            self.sample,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self.record_event({
            'source': 'evaluator',
            'event': 'evaluator_started',
            'sim_time_s': self.started_sim_time_s,
            'wall_time_s': self.started_wall_time_s,
            'state': self.state,
            'ground_truth_source': (
                '/model/pick_object/pose (actual Gazebo model Pose_V)'
            ),
        })

    def record_event(self, data):
        self.events_handle.write(json.dumps(data, sort_keys=True) + '\n')
        self.events_handle.flush()

    def pose_callback(self, message):
        pose = extract_single_model_pose(message)
        if pose is None:
            return
        self.object_pose = pose
        self.object_pose_rx_wall = time.monotonic()
        if self.initial_pose is None:
            self.initial_pose = self.object_pose
        self.max_z = max(self.max_z, self.object_pose[2])

    def odom_callback(self, message):
        p = message.pose.pose.position
        self.base_pose = (float(p.x), float(p.y))
        if self.base_origin is None:
            self.base_origin = self.base_pose
        self.max_base_drift = max(
            self.max_base_drift,
            math.dist(self.base_origin, self.base_pose),
        )

    def attach_callback(self, message):
        value = message.data == 'attached'
        if value and not self.attached:
            self.attach_count += 1
        if not value and self.attached:
            self.detach_count += 1
            self.detached_at = self.get_clock().now().nanoseconds / 1e9
        self.attached = value

    def gate_callback(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.record_event(data)
        if data.get('event') in ('attach_command', 'physical_grasp_verified'):
            self.attach_gate_snapshots.append(data)
            if self.grasp_object_pose is None:
                self.grasp_object_pose = self.object_pose

    def status_callback(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.record_event(data)
        event = data.get('event')
        if event == 'trajectory_feedback':
            error = float(data.get('max_joint_error_rad', 0.0))
            self.max_joint_tracking_error = max(
                self.max_joint_tracking_error, error
            )
        elif event == 'trajectory_sent':
            self.trajectory_goal_count += 1
        elif event == 'trajectory_goal_rejected_retry':
            self.trajectory_retry_count += 1
        previous = self.state
        self.state = str(data.get('state', self.state))
        if data.get('event') == 'transition':
            self.transitions.append(data)
            destination = data.get('to_state')
            sim_time = float(data.get('sim_time_s', 0.0))
            if destination == 'CLOSE':
                self.measure_pregrasp()
            if destination == 'HOLD':
                self.hold_started = sim_time
                self.lift_object_pose = self.object_pose
            if destination == 'TRANSFER' and self.hold_started is not None:
                self.hold_duration = sim_time - self.hold_started
        if data.get('event') == 'terminal' and not self.finalized:
            self.terminal_payload = data
            self.finalize()
        if previous != self.state:
            self.stable_since = None
            self.stability_anchor = None

    def contact_callback(self, message):
        finger_tokens = (
            'poppy_fixed_finger_collision',
            'poppy_moving_finger_collision',
        )
        finger_states = {
            'CLOSE', 'VERIFY_GRASP', 'LIFT', 'HOLD', 'TRANSFER', 'LOWER',
            'RELEASE', 'RETREAT', 'RECOVER',
        }
        pick_support_states = {
            'IDLE', 'FREEZE_BASE', 'OPEN', 'PREGRASP', 'APPROACH', 'CLOSE',
            'VERIFY_GRASP', 'LIFT', 'RECOVER', 'FAILED', 'CANCELLED',
        }
        place_support_states = {
            'LOWER', 'RELEASE', 'RETREAT', 'DONE', 'RECOVER', 'FAILED',
            'CANCELLED',
        }
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if not any('pick_object_collision' in name for name in names):
                continue
            other = names[1] if 'pick_object_collision' in names[0] else names[0]
            allowed = (
                (
                    self.state in finger_states
                    and any(token in other for token in finger_tokens)
                )
                or (
                    'pick_support_collision' in other
                    and self.state in pick_support_states
                )
                or (
                    'place_support_collision' in other
                    and self.state in place_support_states
                )
            )
            if not allowed:
                key = (self.state, other)
                item = {
                    'sim_time_s': self.get_clock().now().nanoseconds / 1e9,
                    'state': self.state,
                    'other_collision': other,
                }
                if key not in self.forbidden_contact_keys:
                    self.forbidden_contact_keys.add(key)
                    self.forbidden_contacts.append(item)

    def lookup(self, frame, timeout_s=0.05):
        tf = self.tf_buffer.lookup_transform(
            'odom', frame, Time(), timeout=Duration(seconds=timeout_s)
        )
        p = tf.transform.translation
        return (float(p.x), float(p.y), float(p.z))

    def measure_pregrasp(self):
        if self.object_pose is None:
            return
        try:
            grasp = self.lookup('poppy_grasp_frame')
            fixed = self.lookup('poppy_fixed_tip')
            moving = self.lookup('poppy_moving_tip')
        except TransformException:
            return
        self.pregrasp_position_error = math.dist(grasp, self.object_pose)
        span = math.dist(fixed, moving)
        self.pregrasp_angle_error = (
            math.degrees(
                math.asin(min(1.0, abs(moving[2] - fixed[2]) / span))
            )
            if span > 1e-6 else math.inf
        )

    def sample(self):
        if self.finalized:
            return
        sim_time = self.get_clock().now().nanoseconds / 1e9
        x = y = z = math.nan
        if self.object_pose is not None:
            x, y, z = self.object_pose
        try:
            self.tool_pose = self.lookup(
                str(self.get_parameter('tool_frame').value), timeout_s=0.0
            )
            self.grasp_pose = self.lookup(
                str(self.get_parameter('grasp_frame').value), timeout_s=0.0
            )
        except TransformException:
            pass
        if self.initial_tool_pose is None and self.tool_pose is not None:
            self.initial_tool_pose = self.tool_pose
        bx = by = math.nan
        if self.base_pose is not None:
            bx, by = self.base_pose
        tx = ty = tz = math.nan
        if self.tool_pose is not None:
            tx, ty, tz = self.tool_pose
        gx = gy = gz = math.nan
        if self.grasp_pose is not None:
            gx, gy, gz = self.grasp_pose
        self.writer.writerow({
            'sim_time_s': sim_time,
            'wall_time_s': time.time(),
            'state': self.state,
            'object_x_m': x,
            'object_y_m': y,
            'object_z_m': z,
            'attached': self.attached,
            'base_x_m': bx,
            'base_y_m': by,
            'tool_x_m': tx,
            'tool_y_m': ty,
            'tool_z_m': tz,
            'grasp_x_m': gx,
            'grasp_y_m': gy,
            'grasp_z_m': gz,
        })
        self.samples_handle.flush()

        if self.detached_at is not None and self.object_pose is not None:
            target = (
                float(self.get_parameter('place_x_m').value),
                float(self.get_parameter('place_y_m').value),
            )
            placement_tolerance = float(
                self.get_parameter('placement_tolerance_m').value
            )
            motion_tolerance = float(
                self.get_parameter('stability_motion_tolerance_m').value
            )
            (
                self.stability_anchor,
                self.stable_since,
                self.stable_duration,
            ) = update_spatial_stability(
                self.stability_anchor,
                self.stable_since,
                self.stable_duration,
                sim_time,
                self.object_pose,
                target,
                placement_tolerance,
                motion_tolerance,
                self.attached,
            )

    def finalize(self):
        self.finalized = True
        initial_z = self.initial_pose[2] if self.initial_pose else math.nan
        lift = self.max_z - initial_z if self.initial_pose else math.nan
        support_clearance = (
            self.max_z
            - float(self.get_parameter('object_height_m').value) / 2.0
            - float(self.get_parameter('pick_support_top_z_m').value)
        )
        target = (
            float(self.get_parameter('place_x_m').value),
            float(self.get_parameter('place_y_m').value),
        )
        placement_error = (
            math.dist(self.object_pose[:2], target)
            if self.object_pose is not None else math.inf
        )
        gate_complete = bool(self.attach_gate_snapshots) and all(
            item.get('snapshot', {}).get(key)
            for item in self.attach_gate_snapshots
            for key in (
                'object_present', 'fixed_contact_fresh',
                'moving_contact_fresh', 'close_commanded',
                'base_stopped', 'tf_valid',
            )
        )
        terminal = str(self.terminal_payload.get('result', 'UNKNOWN'))
        grasp_mode = str(self.get_parameter('grasp_mode').value)
        if grasp_mode == 'attach_conditioned':
            grasp_contract = (
                self.attach_count == 1
                and gate_complete
                and len(self.attach_gate_snapshots) == 1
            )
            released = not self.attached and self.detach_count == 1
            simulator_assisted = True
        else:
            grasp_contract = (
                self.attach_count == 0
                and gate_complete
                and len(self.attach_gate_snapshots) == 1
            )
            released = (
                not self.attached
                and self.attach_count == 0
                and self.detach_count == 0
            )
            simulator_assisted = False
        criteria = {
            'terminal_done': terminal == 'DONE',
            'grasp_contract_verified': grasp_contract,
            'released': released,
            'lift_at_least_50mm': lift >= float(
                self.get_parameter('minimum_lift_m').value
            ),
            'separated_from_pick_support': support_clearance >= float(
                self.get_parameter('minimum_support_clearance_m').value
            ),
            'hold_at_least_3s': self.hold_duration >= float(
                self.get_parameter('minimum_hold_s').value
            ),
            'placement_within_30mm': placement_error <= float(
                self.get_parameter('placement_tolerance_m').value
            ),
            'stable_at_least_2s': self.stable_duration >= float(
                self.get_parameter('minimum_stable_s').value
            ),
            'base_drift': self.max_base_drift <= float(
                self.get_parameter('base_drift_limit_m').value
            ),
            'no_forbidden_contacts': not self.forbidden_contacts,
            'pregrasp_position': (
                self.pregrasp_position_error is not None
                and self.pregrasp_position_error <= 0.010
            ),
            'pregrasp_orientation': (
                self.pregrasp_angle_error is not None
                and self.pregrasp_angle_error <= 5.0
            ),
        }
        result = {
            'run_id': str(self.get_parameter('run_id').value),
            'status': 'passed' if all(criteria.values()) else 'failed',
            'terminal_state': terminal,
            'terminal_reason': self.terminal_payload.get('reason', ''),
            'source_sha': str(self.get_parameter('source_sha').value),
            'source_dirty': bool(self.get_parameter('source_dirty').value),
            'world': str(self.get_parameter('world').value),
            'seed': int(self.get_parameter('seed').value),
            'grasp_mode': grasp_mode,
            'simulator_assisted': simulator_assisted,
            'control_source': 'predefined_joint_trajectories',
            'ground_truth_used_for_control': False,
            'ground_truth_used_for_attachment_gate': True,
            'ground_truth_used_for_evaluation': True,
            'ground_truth_source': (
                '/model/pick_object/pose (actual Gazebo model Pose_V)'
            ),
            'started_wall_time_s': self.started_wall_time_s,
            'completed_wall_time_s': float(
                self.terminal_payload.get('wall_time_s', time.time())
            ),
            'started_sim_time_s': self.started_sim_time_s,
            'completed_sim_time_s': float(
                self.terminal_payload.get(
                    'sim_time_s',
                    self.get_clock().now().nanoseconds / 1e9,
                )
            ),
            'initial_base_pose_xy_m': self.base_origin,
            'final_base_pose_xy_m': self.base_pose,
            'initial_tool_pose_m': self.initial_tool_pose,
            'final_tool_pose_m': self.tool_pose,
            'final_grasp_frame_pose_m': self.grasp_pose,
            'initial_object_pose_m': self.initial_pose,
            'object_pose_at_grasp_m': self.grasp_object_pose,
            'object_pose_at_lift_m': self.lift_object_pose,
            'final_object_pose_m': self.object_pose,
            'maximum_object_z_m': self.max_z,
            'lift_m': lift,
            'maximum_pick_support_clearance_m': support_clearance,
            'stability_motion_tolerance_m': float(
                self.get_parameter('stability_motion_tolerance_m').value
            ),
            'hold_duration_s': self.hold_duration,
            'stable_duration_s': self.stable_duration,
            'placement_error_xy_m': placement_error,
            'pregrasp_position_error_m': self.pregrasp_position_error,
            'pregrasp_relevant_angle_error_deg': self.pregrasp_angle_error,
            'maximum_base_drift_m': self.max_base_drift,
            'maximum_joint_tracking_error_rad': (
                self.max_joint_tracking_error
            ),
            'trajectory_goal_count': self.trajectory_goal_count,
            'trajectory_retry_count': self.trajectory_retry_count,
            'timeout_or_failure_cause': self.terminal_payload.get('reason', ''),
            'attach_count': self.attach_count,
            'detach_count': self.detach_count,
            'attach_authorizations': self.attach_gate_snapshots,
            'forbidden_contacts': self.forbidden_contacts,
            'criteria': criteria,
            'transitions': self.transitions,
            'raw_events': self.events_path.name,
            'raw_samples': self.samples_path.name,
        }
        (self.output / 'run.json').write_text(
            json.dumps(json_safe(result), indent=2, allow_nan=False) + '\n',
            encoding='utf-8',
        )
        self.events_handle.flush()
        self.samples_handle.flush()
        done = Bool()
        done.data = True
        self.result_pub.publish(done)
        self.get_logger().info(json.dumps(result, sort_keys=True))


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.events_handle.close()
        node.samples_handle.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
