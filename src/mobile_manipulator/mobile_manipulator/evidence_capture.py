"""Save one ROS camera frame and its annotated copy as diagnostic evidence."""
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class EvidenceCapture(Node):
    def __init__(self):
        super().__init__('evidence_capture')
        self.bridge = CvBridge()
        self.done = False
        self.output = Path('captures/png')
        self.output.mkdir(parents=True, exist_ok=True)
        self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, message):
        if self.done:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        cv2.imwrite(str(self.output / 'front_camera.png'), image)
        annotated = image.copy()
        cv2.putText(annotated, 'ROS 2 / Gazebo front camera', (16, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
        cv2.imwrite(str(self.output / 'front_camera_annotated.png'), annotated)
        self.done = True
        self.get_logger().info('saved camera evidence')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = EvidenceCapture()
    rclpy.spin(node)
