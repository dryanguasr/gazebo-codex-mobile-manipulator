import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def target_position(
    elapsed_s,
    mode,
    centre_x_m,
    longitudinal_amplitude_m,
    lateral_amplitude_m,
    angular_frequency_rad_s,
):
    """Return a deterministic planar target position."""
    if mode == 'static':
        return centre_x_m, 0.0
    if mode != 'moving':
        raise ValueError(f'Unsupported target mode: {mode}')
    phase = angular_frequency_rad_s * elapsed_s
    x = centre_x_m + longitudinal_amplitude_m * math.sin(phase)
    y = lateral_amplitude_m * math.sin(0.5 * phase)
    return x, y


class TargetTrajectory(Node):
    """Move target_ball through Gazebo's public pose service."""

    def __init__(self):
        super().__init__('target_trajectory')
        self.declare_parameter('mode', 'static')
        self.declare_parameter('entity_name', 'target_ball')
        self.declare_parameter('centre_x_m', 2.0)
        self.declare_parameter('height_m', 0.12)
        self.declare_parameter('longitudinal_amplitude_m', 0.45)
        self.declare_parameter('lateral_amplitude_m', 0.65)
        self.declare_parameter('angular_frequency_rad_s', 0.25)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter(
            'set_pose_service', '/world/ball_arena/set_pose'
        )

        self.client = self.create_client(
            SetEntityPose, self.get_parameter('set_pose_service').value
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/target/ground_truth', 10
        )
        self.start_time = None
        self.pending_request = None
        self.last_pose = None
        period = 1.0 / float(self.get_parameter('update_rate_hz').value)
        self.create_timer(period, self.timer_callback)

    def timer_callback(self):
        if not self.client.service_is_ready():
            self.get_logger().info(
                'Waiting for Gazebo set_pose service...',
                throttle_duration_sec=2.0,
            )
            return
        if self.pending_request is not None and not self.pending_request.done():
            return
        if self.pending_request is not None:
            response = self.pending_request.result()
            if response is None or not response.success:
                self.get_logger().error('Gazebo rejected target pose')
            elif self.last_pose is not None:
                self.pose_publisher.publish(self.last_pose)

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed_s = (now - self.start_time).nanoseconds / 1e9
        x, y = target_position(
            elapsed_s,
            str(self.get_parameter('mode').value),
            float(self.get_parameter('centre_x_m').value),
            float(self.get_parameter('longitudinal_amplitude_m').value),
            float(self.get_parameter('lateral_amplitude_m').value),
            float(self.get_parameter('angular_frequency_rad_s').value),
        )

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = float(self.get_parameter('height_m').value)
        pose.pose.orientation.w = 1.0

        request = SetEntityPose.Request()
        request.entity.name = str(self.get_parameter('entity_name').value)
        request.entity.type = Entity.MODEL
        request.pose = pose.pose
        self.last_pose = pose
        self.pending_request = self.client.call_async(request)


def main():
    rclpy.init()
    node = TargetTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
