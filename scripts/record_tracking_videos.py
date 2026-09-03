#!/usr/bin/env python3
"""Record synchronized 60 fps tracking videos from two ROS image topics."""

import argparse
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


FPS = 60.0


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


class StreamWriter:
    def __init__(self, path, size, total_frames, label):
        self.path = path
        self.size = size
        self.total_frames = total_frames
        self.label = label
        self.writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            FPS,
            size,
        )
        if not self.writer.isOpened():
            raise RuntimeError(f"Could not open video writer for {path}")
        self.frames_written = 0
        self.last_frame = None

    @property
    def done(self):
        return self.frames_written >= self.total_frames

    def add(self, frame, timestamp_ns, start_ns):
        if self.done or timestamp_ns < start_ns:
            return
        target_index = min(
            int(round((timestamp_ns - start_ns) * FPS / 1_000_000_000)),
            self.total_frames - 1,
        )
        if target_index < self.frames_written:
            return

        frame = cv2.resize(frame, self.size, interpolation=cv2.INTER_AREA)
        elapsed_s = target_index / FPS
        lap = min(2, int(elapsed_s / 30.0) + 1)
        bar_y = self.size[1] - 54
        cv2.rectangle(
            frame,
            (0, bar_y),
            (self.size[0], self.size[1]),
            (18, 18, 18),
            -1,
        )
        cv2.putText(
            frame,
            self.label,
            (18, self.size[1] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (245, 245, 245),
            2,
            cv2.LINE_AA,
        )
        status = f"{elapsed_s:05.2f} s | VUELTA {lap}/2 | 60 fps"
        (text_width, _), _ = cv2.getTextSize(
            status, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2
        )
        cv2.putText(
            frame,
            status,
            (self.size[0] - text_width - 18, self.size[1] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (80, 220, 255),
            2,
            cv2.LINE_AA,
        )

        if self.last_frame is None:
            self.last_frame = frame
        while self.frames_written < target_index:
            self.writer.write(self.last_frame)
            self.frames_written += 1
        if not self.done:
            self.writer.write(frame)
            self.frames_written += 1
            self.last_frame = frame

    def finish(self):
        if self.last_frame is not None:
            while self.frames_written < self.total_frames:
                self.writer.write(self.last_frame)
                self.frames_written += 1
        self.writer.release()


class TrackingVideoRecorder(Node):
    def __init__(self, output_dir, duration_s):
        super().__init__("tracking_video_recorder")
        self.bridge = CvBridge()
        self.total_frames = int(round(duration_s * FPS))
        self.writers = {
            "eagle": StreamWriter(
                output_dir / "isometric_source.mp4",
                (1280, 720),
                self.total_frames,
                "VISTA ISOMETRICA | POPPY ERGO JR | RUTA DE 5 LOBULOS",
            ),
            "robot": StreamWriter(
                output_dir / "robot_source.mp4",
                (960, 720),
                self.total_frames,
                "CAMARA DEL ROBOT | POPPY ERGO JR | CONTROL VISUAL",
            ),
        }
        self.latest = {}
        self.start_ns = None
        self.finished = False
        self.create_subscription(
            Image,
            "/eagle/image_raw",
            lambda message: self.handle_image("eagle", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/ball/debug",
            lambda message: self.handle_image("robot", message),
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Waiting for both views; recording {duration_s:.1f} s at 60 fps"
        )

    def handle_image(self, stream, message):
        if self.finished:
            return
        timestamp = stamp_ns(message)
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.latest[stream] = (timestamp, frame)
        if self.start_ns is None:
            if len(self.latest) < 2:
                return
            self.start_ns = max(item[0] for item in self.latest.values())
            self.get_logger().info(
                "Both views ready; synchronized recording started"
            )
        self.writers[stream].add(frame, timestamp, self.start_ns)
        if all(writer.done for writer in self.writers.values()):
            self.finished = True
            self.get_logger().info(
                "Requested frame count captured in both views"
            )
            rclpy.shutdown()

    def close(self):
        for writer in self.writers.values():
            writer.finish()
            self.get_logger().info(
                f"{writer.path}: {writer.frames_written} frames"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = TrackingVideoRecorder(args.output_dir, args.duration)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
