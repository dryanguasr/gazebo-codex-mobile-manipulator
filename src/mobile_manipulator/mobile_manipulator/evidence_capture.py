"""Save one ROS camera frame as reproducible diagnostic evidence."""

from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class EvidenceCapture(Node):
    def __init__(self):
        super().__init__('evidence_capture')
        self.declare_parameter('output_dir', 'captures/png')
        self.bridge = CvBridge()
        self.done = False
        self.output_dir = Path(str(self.get_parameter('output_dir').value))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, message):
        if self.done:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        raw_path = self.output_dir / 'front_camera.png'
        annotated_path = self.output_dir / 'front_camera_annotated.png'
        if not cv2.imwrite(str(raw_path), image):
            raise RuntimeError(f'Could not write {raw_path}')
        annotated = image.copy()
        cv2.putText(
            annotated,
            'ROS 2 / Gazebo front camera',
            (16, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
        )
        if not cv2.imwrite(str(annotated_path), annotated):
            raise RuntimeError(f'Could not write {annotated_path}')
        self.done = True
        self.get_logger().info(f'Camera evidence written to {self.output_dir}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = EvidenceCapture()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
