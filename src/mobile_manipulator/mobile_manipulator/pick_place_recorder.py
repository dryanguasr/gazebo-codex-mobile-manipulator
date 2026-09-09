"""Record real Gazebo camera evidence for a pick-and-place run."""

import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


NANOSECONDS_PER_SECOND = 1_000_000_000


def output_frame_index(stamp_ns, start_ns, fps):
    """Map a simulation timestamp to the nearest constant-rate frame."""
    if fps <= 0.0:
        raise ValueError('fps must be positive')
    elapsed_ns = max(0, stamp_ns - start_ns)
    return int(round(elapsed_ns * fps / NANOSECONDS_PER_SECOND))


def image_stamp_ns(message):
    return (
        message.header.stamp.sec * NANOSECONDS_PER_SECOND
        + message.header.stamp.nanosec
    )


PHASE_BY_DESTINATION = {
    'APPROACH': 'pregrasp',
    'HOLD': 'lift',
    'TRANSFER': 'retention',
    'NAVIGATE': 'transport_start',
    'DOCK_BASE': 'arrival',
    'RELEASE': 'deposit',
    'RETREAT': 'release',
}


class PickPlaceRecorder(Node):
    """Write an MP4 and event-aligned PNGs from the simulation camera."""

    def __init__(self):
        super().__init__('pick_place_recorder')
        self.declare_parameter('output_dir', '/tmp/pick_place_a1')
        self.declare_parameter('fps', 60.0)
        self.declare_parameter('image_topic', '/pick_place/evidence/image')
        self.declare_parameter('mobile_transport', False)
        self.declare_parameter('grasp_mode', 'attach_conditioned')
        self.output = Path(str(self.get_parameter('output_dir').value)) / 'media'
        self.raw_output = self.output / 'raw'
        self.output.mkdir(parents=True, exist_ok=True)
        self.raw_output.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.state = 'IDLE'
        self.last_frame = None
        self.writer = None
        self.frame_count = 0
        self.source_frame_count = 0
        self.start_stamp_ns = None
        self.last_stamp_ns = None
        self.last_encoded_frame = None
        self.captured = {}
        self.closed = False
        self.started_wall = time.time()
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, '/pick_place/status', self.status_callback, 50
        )
        self.create_subscription(
            String, '/pick_place/gate_event', self.gate_callback, 50
        )

    def annotated(self, frame):
        image = frame.copy()
        cv2.rectangle(image, (0, 0), (image.shape[1], 48), (20, 20, 20), -1)
        cv2.putText(
            image,
            f'{self.get_parameter("grasp_mode").value} | {self.state}',
            (14, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (70, 240, 90),
            2,
            cv2.LINE_AA,
        )
        cv2.rectangle(
            image, (0, image.shape[0] - 32),
            (image.shape[1], image.shape[0]), (20, 20, 20), -1,
        )
        cv2.putText(
            image,
            ('REJILLA: 20 cm | AGARRE: UNION TEMPORAL'
             if bool(self.get_parameter('mobile_transport').value)
             else 'FIJA: VERDE | MOVIL: MAGENTA | SERVOMOTORES: GRIS'),
            (14, image.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 230, 230), 1, cv2.LINE_AA,
        )
        return image

    def ensure_writer(self, frame):
        if self.writer is not None:
            return
        height, width = frame.shape[:2]
        path = self.output / 'pick_and_place_a1.mp4'
        self.writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*'mp4v'),
            float(self.get_parameter('fps').value),
            (width, height),
        )
        if not self.writer.isOpened():
            raise RuntimeError(f'Could not create video at {path}')

    def image_callback(self, message):
        if self.closed:
            return
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.source_frame_count += 1
        self.last_frame = frame
        annotated = self.annotated(frame)
        self.ensure_writer(annotated)

        stamp_ns = image_stamp_ns(message)
        if self.start_stamp_ns is None:
            self.start_stamp_ns = stamp_ns
        target_index = output_frame_index(
            stamp_ns,
            self.start_stamp_ns,
            float(self.get_parameter('fps').value),
        )
        if self.last_encoded_frame is not None:
            while self.frame_count < target_index:
                self.writer.write(self.last_encoded_frame)
                self.frame_count += 1
        if self.frame_count <= target_index:
            self.writer.write(annotated)
            self.frame_count += 1
        self.last_encoded_frame = annotated
        self.last_stamp_ns = stamp_ns

    def capture(self, phase):
        if phase in self.captured or self.last_frame is None:
            return
        raw_path = self.raw_output / f'{phase}.png'
        annotated_path = self.output / f'{phase}.png'
        if not cv2.imwrite(str(raw_path), self.last_frame):
            raise RuntimeError(f'Could not write {raw_path}')
        if not cv2.imwrite(str(annotated_path), self.annotated(self.last_frame)):
            raise RuntimeError(f'Could not write {annotated_path}')
        self.captured[phase] = {
            'frame': self.frame_count,
            'state': self.state,
            'raw': str(raw_path.relative_to(self.output)),
            'annotated': annotated_path.name,
        }

    def status_callback(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.state = str(data.get('state', self.state))
        if data.get('event') == 'transition':
            phase = PHASE_BY_DESTINATION.get(str(data.get('to_state')))
            if phase is not None:
                self.capture(phase)
        if data.get('event') == 'terminal':
            self.close()

    def gate_callback(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if data.get('event') in (
            'attach_command', 'physical_grasp_verified'
        ):
            self.capture('contact')

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.writer is not None:
            self.writer.release()
        manifest = {
            'source': str(self.get_parameter('image_topic').value) + ' (Gazebo camera)',
            'video': 'pick_and_place_a1.mp4',
            'frame_count': self.frame_count,
            'source_frame_count': self.source_frame_count,
            'fps': float(self.get_parameter('fps').value),
            'timing_source': 'Gazebo image header stamp',
            'simulated_duration_s': (
                None
                if self.start_stamp_ns is None or self.last_stamp_ns is None
                else (self.last_stamp_ns - self.start_stamp_ns)
                / NANOSECONDS_PER_SECOND
            ),
            'encoded_duration_s': (
                self.frame_count
                / float(self.get_parameter('fps').value)
            ),
            'started_wall_time_s': self.started_wall,
            'completed_wall_time_s': time.time(),
            'captured_phases': self.captured,
            'required_phases': [
                'pregrasp', 'contact', 'lift', 'retention',
                'deposit', 'release',
            ],
        }
        if bool(self.get_parameter('mobile_transport').value):
            manifest['required_phases'] += ['transport_start', 'arrival']
        manifest['complete'] = all(
            phase in self.captured for phase in manifest['required_phases']
        )
        (self.output / 'manifest.json').write_text(
            json.dumps(manifest, indent=2) + '\n',
            encoding='utf-8',
        )

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceRecorder()
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
