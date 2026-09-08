from collections import Counter, deque
import csv
import json
import math
from pathlib import Path
from statistics import mean

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


CSV_FIELDS = [
    'timestamp_s',
    'elapsed_s',
    'valid_detection',
    'horizontal_error',
    'estimated_distance_m',
    'target_distance_m',
    'distance_target_error_m',
    'linear_command_mps',
    'angular_command_radps',
    'robot_x_m',
    'robot_y_m',
    'robot_yaw_rad',
    'target_x_m',
    'target_y_m',
    'reference_valid',
    'reference_failure',
    'reference_frame',
    'target_frame',
    'target_sample_age_s',
    'transform_age_s',
    'reference_camera_range_m',
    'ground_truth_camera_distance_m',
    'estimation_error_m',
]


class ReferenceUnavailable(RuntimeError):
    """A tracking reference could not be built without stale fallbacks."""


def stamp_nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def select_fresh_sample(samples, query_ns, max_age_s):
    """Return the newest non-future sample within the declared age gate."""
    if query_ns <= 0:
        raise ReferenceUnavailable('measurement_stamp_missing')
    if not samples:
        raise ReferenceUnavailable('target_pose_missing')
    eligible = [sample for sample in samples if sample[0] <= query_ns]
    if not eligible:
        raise ReferenceUnavailable('target_pose_from_future')
    sample_ns, sample = max(eligible, key=lambda item: item[0])
    age_s = (query_ns - sample_ns) / 1e9
    if age_s > max_age_s:
        raise ReferenceUnavailable('target_pose_stale')
    return sample, age_s


def _rotate_vector(vector, quaternion):
    """Rotate a vector by a geometry_msgs quaternion."""
    qx, qy, qz, qw = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def reference_range_from_transform(
    target_xyz,
    transform,
    query_ns,
    max_transform_age_s,
):
    """Transform an odom target point into the camera and return its range."""
    if transform is None:
        raise ReferenceUnavailable('transform_unavailable')
    transform_ns = stamp_nanoseconds(transform.header.stamp)
    if transform_ns <= 0:
        raise ReferenceUnavailable('transform_stamp_missing')
    transform_age_s = abs(query_ns - transform_ns) / 1e9
    if transform_age_s > max_transform_age_s:
        raise ReferenceUnavailable('transform_stale')

    rotated = _rotate_vector(target_xyz, transform.transform.rotation)
    translation = transform.transform.translation
    point_camera = (
        rotated[0] + translation.x,
        rotated[1] + translation.y,
        rotated[2] + translation.z,
    )
    return (
        math.sqrt(sum(value * value for value in point_camera)),
        transform_age_s,
    )


def root_mean_square(values):
    if not values:
        return None
    return math.sqrt(mean(value * value for value in values))


def summarize_rows(rows, warmup_s, target_tolerance_m):
    usable = [row for row in rows if row['elapsed_s'] >= warmup_s]
    valid = [row for row in usable if row['valid_detection']]
    estimation_errors = [
        row['estimation_error_m']
        for row in valid
        if math.isfinite(row['estimation_error_m'])
    ]
    horizontal_errors = [
        row['horizontal_error']
        for row in valid
        if math.isfinite(row['horizontal_error'])
    ]
    target_errors = [
        row['distance_target_error_m']
        for row in usable
        if math.isfinite(row['distance_target_error_m'])
    ]
    final_window_start = (
        usable[-1]['elapsed_s'] * 0.75 if usable else float('inf')
    )
    final_target_errors = [
        abs(row['distance_target_error_m'])
        for row in usable
        if row['elapsed_s'] >= final_window_start
        and math.isfinite(row['distance_target_error_m'])
    ]
    active_commands = [
        row
        for row in usable
        if abs(row['linear_command_mps']) > 1e-3
        or abs(row['angular_command_radps']) > 1e-3
    ]
    target_x_values = [
        row['target_x_m']
        for row in usable
        if math.isfinite(row['target_x_m'])
    ]
    target_y_values = [
        row['target_y_m']
        for row in usable
        if math.isfinite(row['target_y_m'])
    ]
    robot_positions = [
        (row['robot_x_m'], row['robot_y_m'])
        for row in usable
        if math.isfinite(row['robot_x_m']) and math.isfinite(row['robot_y_m'])
    ]
    robot_displacement_m = None
    if robot_positions:
        robot_displacement_m = math.hypot(
            robot_positions[-1][0] - robot_positions[0][0],
            robot_positions[-1][1] - robot_positions[0][1],
        )

    reference_rows = [row for row in usable if row.get('reference_valid', True)]
    reference_failures = Counter(
        row.get('reference_failure', '')
        for row in usable
        if not row.get('reference_valid', True)
    )

    settling_time_s = None
    for index, row in enumerate(usable):
        remaining_errors = [
            abs(item['distance_target_error_m'])
            for item in usable[index:]
            if math.isfinite(item['distance_target_error_m'])
        ]
        if remaining_errors and (
            sum(error <= target_tolerance_m for error in remaining_errors)
            / len(remaining_errors)
            >= 0.9
        ):
            settling_time_s = row['elapsed_s']
            break

    return {
        'samples_total': len(rows),
        'samples_after_warmup': len(usable),
        'valid_detections': len(valid),
        'detection_rate_percent': (
            100.0 * len(valid) / len(usable) if usable else 0.0
        ),
        'distance_estimation_mae_m': (
            mean(abs(value) for value in estimation_errors)
            if estimation_errors
            else None
        ),
        'distance_estimation_rmse_m': root_mean_square(estimation_errors),
        'horizontal_error_rms': root_mean_square(horizontal_errors),
        'target_distance_error_mae_m': (
            mean(abs(value) for value in target_errors)
            if target_errors
            else None
        ),
        'steady_state_target_error_mae_m': (
            mean(final_target_errors) if final_target_errors else None
        ),
        'command_active_percent': (
            100.0 * len(active_commands) / len(usable) if usable else 0.0
        ),
        'target_x_span_m': (
            max(target_x_values) - min(target_x_values)
            if target_x_values
            else None
        ),
        'target_y_span_m': (
            max(target_y_values) - min(target_y_values)
            if target_y_values
            else None
        ),
        'robot_displacement_m': robot_displacement_m,
        'reference_valid_samples': len(reference_rows),
        'reference_valid_rate_percent': (
            100.0 * len(reference_rows) / len(usable) if usable else 0.0
        ),
        'reference_failure_counts': dict(sorted(reference_failures.items())),
        'time_to_first_detection_s': (
            valid[0]['elapsed_s'] if valid else None
        ),
        'settling_time_s': settling_time_s,
        'target_tolerance_m': target_tolerance_m,
    }


class MetricsLogger(Node):
    """Record perception, control and evaluation-only ground truth."""

    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('output_dir', '/tmp/mobile_manipulator_metrics')
        self.declare_parameter('run_label', 'run')
        self.declare_parameter('duration_s', 30.0)
        self.declare_parameter('warmup_s', 5.0)
        self.declare_parameter('target_distance_m', 1.2)
        self.declare_parameter('target_tolerance_m', 0.20)
        self.declare_parameter('reference_camera_frame', 'camera_link')
        self.declare_parameter('max_target_age_s', 0.25)
        self.declare_parameter('max_transform_age_s', 0.10)
        self.declare_parameter('transform_timeout_s', 0.05)

        self.rows = []
        self.start_time = self.get_clock().now()
        self.latest_command = TwistStamped()
        self.latest_odom = None
        self.target_history = deque(maxlen=200)
        self.reference_failure_counts = Counter()
        self.finished = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            Vector3Stamped, '/ball/measurement', self.measurement_callback, 20
        )
        self.create_subscription(
            TwistStamped, '/base_controller/cmd_vel', self.command_callback, 20
        )
        self.create_subscription(
            Odometry, '/base_controller/odom', self.odom_callback, 20
        )
        self.create_subscription(
            PoseStamped, '/target/ground_truth', self.target_callback, 20
        )
        self.create_timer(0.2, self.completion_callback)

    def command_callback(self, message):
        self.latest_command = message

    def odom_callback(self, message):
        self.latest_odom = message

    def target_callback(self, message):
        self.target_history.append(
            (stamp_nanoseconds(message.header.stamp), message)
        )

    def tracking_reference(self, measurement):
        query_ns = stamp_nanoseconds(measurement.header.stamp)
        target, target_age_s = select_fresh_sample(
            self.target_history,
            query_ns,
            float(self.get_parameter('max_target_age_s').value),
        )
        target_frame = target.header.frame_id.strip()
        camera_frame = str(
            self.get_parameter('reference_camera_frame').value
        ).strip()
        if not target_frame:
            raise ReferenceUnavailable('target_frame_missing')
        if not camera_frame:
            raise ReferenceUnavailable('camera_frame_missing')
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                target_frame,
                Time(),
                timeout=Duration(
                    seconds=float(
                        self.get_parameter('transform_timeout_s').value
                    )
                ),
            )
        except TransformException as error:
            raise ReferenceUnavailable('transform_unavailable') from error
        target_position = target.pose.position
        reference_range, transform_age_s = reference_range_from_transform(
            (target_position.x, target_position.y, target_position.z),
            transform,
            query_ns,
            float(self.get_parameter('max_transform_age_s').value),
        )
        return {
            'range_m': reference_range,
            'target_age_s': target_age_s,
            'transform_age_s': transform_age_s,
            'camera_frame': camera_frame,
            'target_frame': target_frame,
            'target_x_m': target_position.x,
            'target_y_m': target_position.y,
        }

    def measurement_callback(self, message):
        now = self.get_clock().now()
        elapsed_s = (now - self.start_time).nanoseconds / 1e9
        valid = math.isfinite(message.vector.z)
        robot_x = robot_y = robot_yaw = math.nan
        target_x = target_y = reference_range = math.nan
        target_age_s = transform_age_s = math.nan
        reference_valid = False
        reference_failure = ''
        reference_frame = str(
            self.get_parameter('reference_camera_frame').value
        )
        target_frame = ''

        if self.latest_odom is not None:
            robot_pose = self.latest_odom.pose.pose
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y
            robot_yaw = self._yaw_from_quaternion(robot_pose.orientation)
        try:
            reference = self.tracking_reference(message)
            reference_range = reference['range_m']
            target_age_s = reference['target_age_s']
            transform_age_s = reference['transform_age_s']
            reference_frame = reference['camera_frame']
            target_frame = reference['target_frame']
            target_x = reference['target_x_m']
            target_y = reference['target_y_m']
            reference_valid = True
        except ReferenceUnavailable as error:
            reference_failure = str(error)
            self.reference_failure_counts[reference_failure] += 1

        target_distance = float(self.get_parameter('target_distance_m').value)
        estimated_distance = message.vector.z if valid else math.nan
        self.rows.append(
            {
                'timestamp_s': now.nanoseconds / 1e9,
                'elapsed_s': elapsed_s,
                'valid_detection': valid,
                'horizontal_error': message.vector.x if valid else math.nan,
                'estimated_distance_m': estimated_distance,
                'target_distance_m': target_distance,
                'distance_target_error_m': (
                    reference_range - target_distance
                    if math.isfinite(reference_range)
                    else math.nan
                ),
                'linear_command_mps': self.latest_command.twist.linear.x,
                'angular_command_radps': self.latest_command.twist.angular.z,
                'robot_x_m': robot_x,
                'robot_y_m': robot_y,
                'robot_yaw_rad': robot_yaw,
                'target_x_m': target_x,
                'target_y_m': target_y,
                'reference_valid': reference_valid,
                'reference_failure': reference_failure,
                'reference_frame': reference_frame,
                'target_frame': target_frame,
                'target_sample_age_s': target_age_s,
                'transform_age_s': transform_age_s,
                'reference_camera_range_m': reference_range,
                # Kept for CSV compatibility; metadata states its provenance.
                'ground_truth_camera_distance_m': reference_range,
                'estimation_error_m': (
                    estimated_distance - reference_range
                    if valid and math.isfinite(reference_range)
                    else math.nan
                ),
            }
        )

    def completion_callback(self):
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed_s >= float(self.get_parameter('duration_s').value):
            self.finalize()
            rclpy.shutdown()

    def finalize(self):
        if self.finished:
            return
        self.finished = True
        output_dir = Path(str(self.get_parameter('output_dir').value))
        run_label = str(self.get_parameter('run_label').value)
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / f'{run_label}.csv'
        summary_path = output_dir / f'{run_label}_summary.json'
        text_path = output_dir / f'{run_label}_summary.txt'

        with csv_path.open('w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(
                handle, fieldnames=CSV_FIELDS, lineterminator='\n'
            )
            writer.writeheader()
            writer.writerows(self.rows)

        summary = summarize_rows(
            self.rows,
            float(self.get_parameter('warmup_s').value),
            float(self.get_parameter('target_tolerance_m').value),
        )
        summary['run_label'] = run_label
        summary['duration_s'] = float(self.get_parameter('duration_s').value)
        summary['tracking_reference'] = {
            'geometry_source': 'tf',
            'camera_frame': str(
                self.get_parameter('reference_camera_frame').value
            ),
            'target_pose_source': 'accepted_set_pose_command',
            'target_pose_frame': 'from_message',
            'measurement_timestamp_source': '/ball/measurement.header.stamp',
            'transform_selection': 'latest_available_with_age_gate',
            'uses_odometry_tf': True,
            'independent_simulator_ground_truth': False,
            'ground_truth_used_for_control': False,
            'max_target_age_s': float(
                self.get_parameter('max_target_age_s').value
            ),
            'max_transform_age_s': float(
                self.get_parameter('max_transform_age_s').value
            ),
        }
        with summary_path.open('w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2, allow_nan=False)
            handle.write('\n')
        with text_path.open('w', encoding='utf-8') as handle:
            for key, value in summary.items():
                handle.write(f'{key}: {value}\n')
        self.get_logger().info(f'Metrics written to {output_dir}')

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)


def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
