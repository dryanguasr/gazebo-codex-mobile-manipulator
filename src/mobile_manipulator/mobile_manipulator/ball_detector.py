import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def focal_length_from_fov(image_width_px, horizontal_fov_rad):
    """Return focal length in pixels for a pinhole camera."""
    return image_width_px / (2.0 * math.tan(horizontal_fov_rad / 2.0))


def estimate_sphere_distance(radius_px, focal_length_px, sphere_radius_m):
    """Estimate optical-axis depth from the apparent angular radius."""
    if radius_px <= 0.0 or focal_length_px <= 0.0 or sphere_radius_m <= 0.0:
        return math.nan
    angular_radius = math.atan(radius_px / focal_length_px)
    return sphere_radius_m / math.sin(angular_radius)


def depth_to_range(
    optical_depth_m,
    pixel_x,
    pixel_y,
    focal_x_px,
    focal_y_px,
    principal_x_px,
    principal_y_px,
):
    """Convert pinhole optical depth to camera-to-point Euclidean range."""
    ray_x = (pixel_x - principal_x_px) / focal_x_px
    ray_y = (pixel_y - principal_y_px) / focal_y_px
    return optical_depth_m * math.sqrt(1.0 + ray_x**2 + ray_y**2)


class BallDetector(Node):
    """Detect the red target using only camera images and camera intrinsics."""

    def __init__(self):
        super().__init__('ball_detector')
        self.declare_parameter('sphere_radius_m', 0.12)
        self.declare_parameter('min_radius_px', 4.0)
        self.declare_parameter('min_saturation', 120)
        self.declare_parameter('min_value', 80)
        self.declare_parameter('red_hue_low_max', 12)
        self.declare_parameter('red_hue_high_min', 170)
        self.declare_parameter('horizontal_fov_rad', 1.047)

        self.bridge = CvBridge()
        self.focal_length_px = None
        self.focal_y_px = None
        self.principal_x_px = None
        self.principal_y_px = None
        self.focal_source = 'unavailable'
        self.warned_about_fallback = False

        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.measurement_publisher = self.create_publisher(
            Vector3Stamped, '/ball/measurement', 10
        )
        self.debug_publisher = self.create_publisher(Image, '/ball/debug', 10)

    def camera_info_callback(self, message):
        if message.k[0] > 0.0:
            first_calibration = self.focal_source != 'camera_info'
            self.focal_length_px = float(message.k[0])
            self.focal_y_px = float(message.k[4])
            self.principal_x_px = float(message.k[2])
            self.principal_y_px = float(message.k[5])
            self.focal_source = 'camera_info'
            if first_calibration:
                self.get_logger().info(
                    f'Using CameraInfo focal length fx={self.focal_length_px:.3f} px'
                )

    def image_callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        focal_x, focal_y, principal_x, principal_y = (
            self._intrinsics_for_image(image.shape[1], image.shape[0])
        )

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        min_saturation = int(self.get_parameter('min_saturation').value)
        min_value = int(self.get_parameter('min_value').value)
        low_hue_max = int(self.get_parameter('red_hue_low_max').value)
        high_hue_min = int(self.get_parameter('red_hue_high_min').value)
        low_red = cv2.inRange(
            hsv,
            np.array([0, min_saturation, min_value]),
            np.array([low_hue_max, 255, 255]),
        )
        high_red = cv2.inRange(
            hsv,
            np.array([high_hue_min, min_saturation, min_value]),
            np.array([180, 255, 255]),
        )
        mask = low_red | high_red
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        measurement = Vector3Stamped()
        measurement.header = message.header
        measurement.vector.x = math.nan
        measurement.vector.y = math.nan
        measurement.vector.z = math.nan
        annotated = image.copy()

        if contours:
            contour = max(contours, key=cv2.contourArea)
            (centre_x, centre_y), radius_px = cv2.minEnclosingCircle(contour)
            min_radius_px = float(self.get_parameter('min_radius_px').value)
            if radius_px >= min_radius_px:
                measurement.vector.x = (
                    centre_x - image.shape[1] / 2.0
                ) / (image.shape[1] / 2.0)
                measurement.vector.y = (
                    centre_y - image.shape[0] / 2.0
                ) / (image.shape[0] / 2.0)
                optical_depth_m = estimate_sphere_distance(
                    radius_px,
                    focal_x,
                    float(self.get_parameter('sphere_radius_m').value),
                )
                measurement.vector.z = depth_to_range(
                    optical_depth_m,
                    centre_x,
                    centre_y,
                    focal_x,
                    focal_y,
                    principal_x,
                    principal_y,
                )
                cv2.circle(
                    annotated,
                    (int(centre_x), int(centre_y)),
                    int(radius_px),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated,
                    f'range={measurement.vector.z:.2f} m',
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

        self.measurement_publisher.publish(measurement)
        debug_message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

    def _intrinsics_for_image(self, image_width_px, image_height_px):
        if self.focal_length_px is not None:
            return (
                self.focal_length_px,
                self.focal_y_px,
                self.principal_x_px,
                self.principal_y_px,
            )
        fallback = focal_length_from_fov(
            image_width_px,
            float(self.get_parameter('horizontal_fov_rad').value),
        )
        if not self.warned_about_fallback:
            self.get_logger().warning(
                'CameraInfo has not arrived; deriving fx from image width and '
                f'horizontal_fov_rad: fx={fallback:.3f} px'
            )
            self.warned_about_fallback = True
        self.focal_source = 'fov_fallback'
        return (
            fallback,
            fallback,
            image_width_px / 2.0,
            image_height_px / 2.0,
        )


def main():
    rclpy.init()
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
