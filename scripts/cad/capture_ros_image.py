#!/usr/bin/env python3
"""Save one ROS image topic frame as PNG for reproducible visual evidence."""

from __future__ import annotations

import argparse
from pathlib import Path
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class OneFrameCapture(Node):
    def __init__(self, topic: str, output: Path, skip: int) -> None:
        super().__init__('cad_one_frame_capture')
        self.bridge = CvBridge()
        self.output = output
        self.skip = skip
        self.received = 0
        self.done = threading.Event()
        self.create_subscription(
            Image,
            topic,
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_image(self, message: Image) -> None:
        if self.done.is_set():
            return
        self.received += 1
        if self.received <= self.skip:
            return
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.output), frame):
            raise RuntimeError(f'could not write {self.output}')
        self.get_logger().info(
            f'saved {frame.shape[1]}x{frame.shape[0]} frame to {self.output}'
        )
        self.done.set()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/eagle/image_raw')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--skip', type=int, default=5)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = OneFrameCapture(args.topic, args.output, args.skip)
    timer = threading.Timer(args.timeout, rclpy.shutdown)
    timer.start()
    try:
        while rclpy.ok() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not node.done.is_set():
        raise TimeoutError(
            f'no usable frame arrived on {args.topic} in {args.timeout:.1f} s'
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
