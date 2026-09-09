import json
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


AUTHORIZED_ATTACH_STATE = 'VERIFY_GRASP'
AUTHORIZED_DETACH_STATE = 'RELEASE'
RETENTION_STATES = ('LIFT', 'HOLD', 'TRANSFER', 'LOWER')


def extract_single_model_pose(message):
    """Return the model world pose from the model-specific Pose_V topic.

    ros_gz_bridge does not preserve Gazebo Pose names in TFMessage on Jazzy,
    but PosePublisher emits the one link-local transform followed by the model
    world transform for this one-link object.
    """
    if len(message.transforms) != 2:
        return None
    p = message.transforms[-1].transform.translation
    return (float(p.x), float(p.y), float(p.z))


def cylinder_tilt_deg(rotation):
    values = (rotation.x, rotation.y, rotation.z, rotation.w)
    norm2 = sum(value * value for value in values)
    if not all(math.isfinite(value) for value in values) or norm2 < 1e-12:
        return math.inf
    cosine = 1.0 - 2.0 * (rotation.x ** 2 + rotation.y ** 2) / norm2
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def evaluate_attach_gate(snapshot, limits):
    """Pure, fail-closed A1 authorization contract."""
    reasons = []
    for key in ('geometry_error_m', 'geometry_angle_deg', 'm6_rad'):
        if not math.isfinite(snapshot.get(key, math.inf)):
            reasons.append('nonfinite_' + key)
    if snapshot.get('state') != AUTHORIZED_ATTACH_STATE:
        reasons.append('state_not_authorized')
    if not snapshot.get('object_present'):
        reasons.append('object_missing')
    if not snapshot.get('fixed_contact_fresh'):
        reasons.append('fixed_contact_missing_or_stale')
    if not snapshot.get('moving_contact_fresh'):
        reasons.append('moving_contact_missing_or_stale')
    if snapshot.get('contact_persistence_s', 0.0) < limits['contact_persistence_s']:
        reasons.append('bilateral_contact_not_persistent')
    if snapshot.get('geometry_error_m', math.inf) > limits['position_tolerance_m']:
        reasons.append('geometry_position')
    if snapshot.get('geometry_angle_deg', math.inf) > limits['angle_tolerance_deg']:
        reasons.append('geometry_orientation')
    if not snapshot.get('close_commanded'):
        reasons.append('close_not_commanded')
    if snapshot.get('m6_rad', math.inf) > limits['close_feedback_max_rad']:
        reasons.append('close_feedback')
    if not snapshot.get('base_stopped'):
        reasons.append('base_not_stopped')
    if not snapshot.get('tf_valid'):
        reasons.append('tf_invalid')
    return not reasons, reasons


class PickPlaceAttachGate(Node):
    def __init__(self):
        super().__init__('pick_place_attach_gate')
        for name, value in (
            ('object_pose_max_age_s', 0.10),
            ('contact_max_age_s', 0.10),
            ('contact_persistence_s', 0.15),
            ('geometry_position_tolerance_m', 0.020),
            ('geometry_angle_tolerance_deg', 5.0),
            ('transform_max_age_s', 0.10),
            ('close_feedback_max_rad', 0.20),
            ('base_speed_limit_mps', 0.01),
            ('attach_enabled', True),
            ('object_name', 'pick_object'),
            ('object_collision_token', 'pick_object_collision'),
            ('fixed_collision_token', 'poppy_fixed_finger_collision'),
            ('moving_collision_token', 'poppy_moving_finger_collision'),
            ('grasp_frame', 'poppy_grasp_frame'),
            ('fixed_tip_frame', 'poppy_fixed_contact'),
            ('moving_tip_frame', 'poppy_moving_contact'),
        ):
            self.declare_parameter(name, value)

        self.state = 'IDLE'
        self.close_commanded = False
        self.m6_rad = math.inf
        self.base_stopped = False
        self.object_pose = None
        self.object_tilt_deg = math.inf
        self.object_pose_rx_ns = 0
        self.fixed_contact_ns = 0
        self.moving_contact_ns = 0
        self.bilateral_since_ns = 0
        self.attached = False
        self.attach_sent = False
        self.detach_sent = False
        self.last_reasons = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.attach_pub = self.create_publisher(Empty, '/pick_object/attach', 10)
        self.physical_pub = self.create_publisher(
            Bool, '/pick_place/physical_grasp_verified', 10
        )
        self.detach_pub = self.create_publisher(Empty, '/pick_object/detach', 10)
        self.event_pub = self.create_publisher(String, '/pick_place/gate_event', 20)
        self.create_subscription(
            String, '/pick_place/gate_command', self.command_callback, 20
        )
        self.create_subscription(
            Contacts, '/pick_object/contacts', self.contacts_callback, 50
        )
        self.create_subscription(
            TFMessage,
            '/model/pick_object/pose',
            self.pose_callback,
            20,
        )
        self.create_subscription(
            JointState, '/joint_states', self.joints_callback, 20
        )
        self.create_subscription(
            Odometry, '/base_controller/odom', self.odom_callback, 20
        )
        self.create_subscription(
            String, '/pick_object/joint_state', self.attached_callback, 20
        )
        self.create_timer(
            0.02,
            self.tick,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def emit(self, event, **fields):
        payload = {
            'source': 'attach_gate',
            'event': event,
            'state': self.state,
            'sim_time_s': self.now_ns() / 1e9,
            'wall_time_s': time.time(),
            **fields,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self.event_pub.publish(message)

    def command_callback(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            self.emit('command_rejected', reason='invalid_json')
            return
        previous = self.state
        self.state = str(data.get('state', ''))
        self.close_commanded = bool(data.get('close_commanded', False))
        if self.state != previous:
            self.emit('gate_state_changed', from_state=previous)
        if (
            self.state != AUTHORIZED_ATTACH_STATE
            and self.state not in RETENTION_STATES
        ):
            self.attach_sent = False
            self.bilateral_since_ns = 0
        if self.state != AUTHORIZED_DETACH_STATE:
            self.detach_sent = False

    def joints_callback(self, message):
        try:
            index = message.name.index('poppy_m6_joint')
        except ValueError:
            return
        if index < len(message.position):
            self.m6_rad = float(message.position[index])

    def odom_callback(self, message):
        twist = message.twist.twist
        speed = math.sqrt(
            twist.linear.x ** 2
            + twist.linear.y ** 2
            + twist.linear.z ** 2
            + twist.angular.x ** 2
            + twist.angular.y ** 2
            + twist.angular.z ** 2
        )
        self.base_stopped = speed <= float(
            self.get_parameter('base_speed_limit_mps').value
        )

    def attached_callback(self, message):
        previous = self.attached
        self.attached = message.data == 'attached'
        if self.attached != previous:
            self.emit('joint_state_changed', attached=self.attached)

    def pose_callback(self, message):
        pose = extract_single_model_pose(message)
        if pose is None:
            return
        self.object_pose = pose
        q = message.transforms[-1].transform.rotation
        self.object_tilt_deg = cylinder_tilt_deg(q)
        self.object_pose_rx_ns = self.now_ns()

    def contacts_callback(self, message):
        now_ns = self.now_ns()
        object_token = str(
            self.get_parameter('object_collision_token').value
        )
        fixed_token = str(self.get_parameter('fixed_collision_token').value)
        moving_token = str(
            self.get_parameter('moving_collision_token').value
        )
        for contact in message.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if not any(object_token in name for name in names):
                continue
            other = names[1] if object_token in names[0] else names[0]
            if fixed_token in other:
                self.fixed_contact_ns = now_ns
            if moving_token in other:
                self.moving_contact_ns = now_ns

    def lookup_xyz(self, frame):
        transform = self.tf_buffer.lookup_transform(
            'odom',
            frame,
            Time(),
            timeout=Duration(seconds=0.02),
        )
        stamp_ns = Time.from_msg(transform.header.stamp).nanoseconds
        age_s = abs(self.now_ns() - stamp_ns) / 1e9
        if stamp_ns <= 0 or age_s > float(
            self.get_parameter('transform_max_age_s').value
        ):
            raise TransformException('stale transform')
        p = transform.transform.translation
        return (float(p.x), float(p.y), float(p.z)), age_s

    def geometry(self):
        grasp, grasp_age = self.lookup_xyz(
            str(self.get_parameter('grasp_frame').value)
        )
        fixed, fixed_age = self.lookup_xyz(
            str(self.get_parameter('fixed_tip_frame').value)
        )
        moving, moving_age = self.lookup_xyz(
            str(self.get_parameter('moving_tip_frame').value)
        )
        error = math.dist(grasp, self.object_pose)
        span = math.dist(fixed, moving)
        if span < 1e-6:
            angle = math.inf
        else:
            angle = math.degrees(
                math.asin(min(1.0, abs(moving[2] - fixed[2]) / span))
            )
        return error, angle, max(grasp_age, fixed_age, moving_age), grasp

    def tick(self):
        if self.state == AUTHORIZED_DETACH_STATE and self.attached:
            if not self.detach_sent:
                self.detach_pub.publish(Empty())
                self.detach_sent = True
                self.emit('detach_command', authorized_state=self.state)
            return

        monitoring = (
            not bool(self.get_parameter('attach_enabled').value)
            and self.state in RETENTION_STATES
        )
        if not monitoring and (
            self.state != AUTHORIZED_ATTACH_STATE or self.attached
        ):
            return
        now_ns = self.now_ns()
        contact_age_limit = float(
            self.get_parameter('contact_max_age_s').value
        )
        object_fresh = (
            self.object_pose is not None
            and (now_ns - self.object_pose_rx_ns) / 1e9 <= float(
                self.get_parameter('object_pose_max_age_s').value
            )
        )
        tf_valid = False
        geometry_error = math.inf
        geometry_angle = math.inf
        grasp = None
        try:
            if object_fresh:
                geometry_error, geometry_angle, _, grasp = self.geometry()
                tf_valid = True
        except TransformException:
            pass
        fixed_contact_fresh = (
            self.fixed_contact_ns > 0
            and (now_ns - self.fixed_contact_ns) / 1e9 <= contact_age_limit
        )
        moving_contact_fresh = (
            self.moving_contact_ns > 0
            and (now_ns - self.moving_contact_ns) / 1e9 <= contact_age_limit
        )
        if fixed_contact_fresh and moving_contact_fresh:
            if self.bilateral_since_ns == 0:
                self.bilateral_since_ns = now_ns
        else:
            self.bilateral_since_ns = 0

        if monitoring:
            # A verification event is not a permanent guarantee of retention.
            retained = (
                object_fresh and tf_valid
                and fixed_contact_fresh and moving_contact_fresh
                and geometry_error <= 0.008
                and self.object_tilt_deg <= 10.0
                and self.close_commanded and self.m6_rad <= 0.20
            )
            self.physical_pub.publish(Bool(data=retained))
            key = ('retained', retained)
            if key != self.last_reasons:
                self.last_reasons = key
                self.emit(
                    'retention_changed', retained=retained,
                    geometry_error_m=geometry_error,
                    object_tilt_deg=self.object_tilt_deg,
                    fixed_contact_fresh=fixed_contact_fresh,
                    moving_contact_fresh=moving_contact_fresh,
                    ground_truth_used_for_safety=True,
                )
            return

        snapshot = {
            'state': self.state,
            'object_present': object_fresh,
            'fixed_contact_fresh': fixed_contact_fresh,
            'moving_contact_fresh': moving_contact_fresh,
            'contact_persistence_s': (
                (now_ns - self.bilateral_since_ns) / 1e9
                if self.bilateral_since_ns else 0.0
            ),
            'geometry_error_m': geometry_error,
            'geometry_angle_deg': geometry_angle,
            'close_commanded': self.close_commanded,
            'm6_rad': self.m6_rad,
            'base_stopped': self.base_stopped,
            'tf_valid': tf_valid,
        }
        limits = {
            'contact_persistence_s': float(
                self.get_parameter('contact_persistence_s').value
            ),
            'position_tolerance_m': float(
                self.get_parameter('geometry_position_tolerance_m').value
            ),
            'angle_tolerance_deg': float(
                self.get_parameter('geometry_angle_tolerance_deg').value
            ),
            'close_feedback_max_rad': float(
                self.get_parameter('close_feedback_max_rad').value
            ),
        }
        allowed, reasons = evaluate_attach_gate(snapshot, limits)
        reason_key = tuple(reasons)
        if reason_key != self.last_reasons:
            self.last_reasons = reason_key
            self.emit('gate_evaluated', allowed=allowed, reasons=reasons, **snapshot)
        if allowed and not self.attach_sent:
            self.attach_sent = True
            event = 'attach_command'
            if bool(self.get_parameter('attach_enabled').value):
                self.attach_pub.publish(Empty())
            else:
                event = 'physical_grasp_verified'
                verified = Bool()
                verified.data = True
                self.physical_pub.publish(verified)
            self.emit(
                event,
                authorized_state=self.state,
                contract=(
                    'bilateral_fresh_contact+geometry+closure+'
                    'authorized_state+base_stopped+tf'
                ),
                snapshot=snapshot,
                grasp_xyz_m=grasp,
                ground_truth_used_for_attachment_gate=True,
                attach_enabled=bool(
                    self.get_parameter('attach_enabled').value
                ),
            )


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceAttachGate()
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
