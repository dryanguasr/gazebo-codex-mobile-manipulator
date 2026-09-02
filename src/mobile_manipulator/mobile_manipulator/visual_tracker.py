import math

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from rclpy.node import Node


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class VisualTracker(Node):
    """Proportional image-based controller with explicit parameters."""

    def __init__(self):
        super().__init__('visual_tracker')
        self.declare_parameter('target_distance_m', 1.2)
        self.declare_parameter('linear_gain', 1.1)
        self.declare_parameter('angular_gain', 2.6)
        self.declare_parameter('max_linear_speed_mps', 0.90)
        self.declare_parameter('max_angular_speed_radps', 2.5)
        self.declare_parameter('distance_deadband_m', 0.04)
        self.declare_parameter('horizontal_deadband', 0.02)
        self.declare_parameter('alignment_slowdown', 0.6)
        self.declare_parameter('measurement_timeout_s', 0.3)

        self.command_publisher = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 10
        )
        self.create_subscription(
            Vector3Stamped, '/ball/measurement', self.measurement_callback, 10
        )
        self.last_measurement_time = None
        self.create_timer(0.1, self.watchdog_callback)

    def measurement_callback(self, measurement):
        self.last_measurement_time = self.get_clock().now()
        command = TwistStamped()
        command.header.stamp = self.last_measurement_time.to_msg()
        command.header.frame_id = 'base_footprint'

        if math.isfinite(measurement.vector.z):
            horizontal_error = measurement.vector.x
            distance_error = (
                measurement.vector.z
                - float(self.get_parameter('target_distance_m').value)
            )
            if abs(horizontal_error) < float(
                self.get_parameter('horizontal_deadband').value
            ):
                horizontal_error = 0.0
            if abs(distance_error) < float(
                self.get_parameter('distance_deadband_m').value
            ):
                distance_error = 0.0

            max_angular = float(
                self.get_parameter('max_angular_speed_radps').value
            )
            command.twist.angular.z = clamp(
                -float(self.get_parameter('angular_gain').value)
                * horizontal_error,
                -max_angular,
                max_angular,
            )
            max_linear = float(self.get_parameter('max_linear_speed_mps').value)
            alignment_scale = 1.0 - min(
                float(self.get_parameter('alignment_slowdown').value),
                abs(horizontal_error),
            )
            command.twist.linear.x = clamp(
                float(self.get_parameter('linear_gain').value)
                * distance_error
                * alignment_scale,
                -max_linear,
                max_linear,
            )

        self.command_publisher.publish(command)

    def watchdog_callback(self):
        if self.last_measurement_time is None:
            return
        age_s = (
            self.get_clock().now() - self.last_measurement_time
        ).nanoseconds / 1e9
        if age_s > float(self.get_parameter('measurement_timeout_s').value):
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_footprint'
            self.command_publisher.publish(stop)
            self.last_measurement_time = None


def main():
    rclpy.init()
    node = VisualTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
