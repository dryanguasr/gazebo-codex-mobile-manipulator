import csv
import json
import math
from pathlib import Path
from statistics import mean

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


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
    'ground_truth_camera_distance_m',
    'estimation_error_m',
]


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
        self.declare_parameter('camera_offset_x_m', 0.38)
        self.declare_parameter('camera_height_m', 0.51)
        self.declare_parameter('ball_height_m', 0.12)

        self.rows = []
        self.start_time = self.get_clock().now()
        self.latest_command = TwistStamped()
        self.latest_odom = None
        self.latest_target = None
        self.finished = False

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
        self.latest_target = message

    def measurement_callback(self, message):
        now = self.get_clock().now()
        elapsed_s = (now - self.start_time).nanoseconds / 1e9
        valid = math.isfinite(message.vector.z)
        robot_x = robot_y = robot_yaw = math.nan
        target_x = target_y = ground_truth_distance = math.nan

        if self.latest_odom is not None:
            robot_pose = self.latest_odom.pose.pose
            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y
            robot_yaw = self._yaw_from_quaternion(robot_pose.orientation)
        if self.latest_target is not None:
            target_x = self.latest_target.pose.position.x
            target_y = self.latest_target.pose.position.y
        if math.isfinite(robot_yaw) and math.isfinite(target_x):
            camera_offset = float(
                self.get_parameter('camera_offset_x_m').value
            )
            camera_x = robot_x + camera_offset * math.cos(robot_yaw)
            camera_y = robot_y + camera_offset * math.sin(robot_yaw)
            vertical_offset = (
                float(self.get_parameter('camera_height_m').value)
                - float(self.get_parameter('ball_height_m').value)
            )
            ground_truth_distance = math.sqrt(
                (target_x - camera_x) ** 2
                + (target_y - camera_y) ** 2
                + vertical_offset**2
            )

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
                    ground_truth_distance - target_distance
                    if math.isfinite(ground_truth_distance)
                    else math.nan
                ),
                'linear_command_mps': self.latest_command.twist.linear.x,
                'angular_command_radps': self.latest_command.twist.angular.z,
                'robot_x_m': robot_x,
                'robot_y_m': robot_y,
                'robot_yaw_rad': robot_yaw,
                'target_x_m': target_x,
                'target_y_m': target_y,
                'ground_truth_camera_distance_m': ground_truth_distance,
                'estimation_error_m': (
                    estimated_distance - ground_truth_distance
                    if valid and math.isfinite(ground_truth_distance)
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
