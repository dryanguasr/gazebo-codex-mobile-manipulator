"""Reset and verify DetachableJoint before a measured pick-and-place run."""

from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_msgs.msg import TFMessage


def reset_ready(
    expect_detachable_joint, joint_state, free_duration_s, pose_fresh
):
    """Return true only after verified detachment and a fresh object pose."""
    return (
        (not expect_detachable_joint or joint_state == 'detached')
        and free_duration_s >= 0.25
        and pose_fresh
    )


class PickPlaceInitializer(Node):
    """Record the plugin reset outside the measured state-machine run."""

    def __init__(self):
        super().__init__('pick_place_initializer')
        for name, value in (
            ('output_dir', '/tmp/pick_place_a1'),
            ('run_id', 'run'),
            ('source_sha', 'unknown'),
            ('source_dirty', False),
            ('seed', 1),
            ('timeout_wall_s', 8.0),
            ('pose_max_age_wall_s', 0.25),
            ('expect_detachable_joint', True),
        ):
            self.declare_parameter(name, value)

        self.output = Path(str(self.get_parameter('output_dir').value))
        self.output.mkdir(parents=True, exist_ok=True)
        self.started_wall = time.time()
        self.started_monotonic = time.monotonic()
        self.last_detach_wall = -math.inf
        self.detach_commands = 0
        self.joint_state = None
        self.initial_joint_state = None
        self.detached_since = None
        self.observed_joint_states = []
        self.object_pose = None
        self.object_pose_wall = -math.inf
        self.pose_history = deque(maxlen=200)
        self.finished = False
        self.success = False
        self.reason = ''

        self.detach_pub = self.create_publisher(
            Empty, '/pick_object/detach', 10
        )
        self.create_subscription(
            String,
            '/pick_object/joint_state',
            self.joint_state_callback,
            20,
        )
        self.create_subscription(
            TFMessage,
            '/model/pick_object/pose',
            self.pose_callback,
            20,
        )
        self.create_timer(
            0.02,
            self.tick,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )

    def joint_state_callback(self, message):
        state = str(message.data).strip().lower()
        if self.initial_joint_state is None:
            self.initial_joint_state = state
        if not self.observed_joint_states or (
            self.observed_joint_states[-1] != state
        ):
            self.observed_joint_states.append(state)
        self.joint_state = state
        if state == 'detached':
            if self.detached_since is None:
                self.detached_since = time.monotonic()
        else:
            self.detached_since = None

    def pose_callback(self, message):
        if len(message.transforms) != 2:
            return
        point = message.transforms[-1].transform.translation
        self.object_pose = (
            float(point.x),
            float(point.y),
            float(point.z),
        )
        self.object_pose_wall = time.monotonic()
        self.pose_history.append(self.object_pose)

    def motion_during_reset(self):
        if len(self.pose_history) < 2:
            return None
        anchor = self.pose_history[0]
        return max(math.dist(anchor, pose) for pose in self.pose_history)

    def finish(self, success, reason):
        if self.finished:
            return
        self.finished = True
        self.success = success
        self.reason = reason
        payload = {
            'status': 'passed' if success else 'failed',
            'reason': reason,
            'phase': 'INITIALIZE_RESET',
            'excluded_from_measured_run': True,
            'run_id': str(self.get_parameter('run_id').value),
            'source_sha': str(self.get_parameter('source_sha').value),
            'source_dirty': bool(
                self.get_parameter('source_dirty').value
            ),
            'seed': int(self.get_parameter('seed').value),
            'started_wall_time_s': self.started_wall,
            'completed_wall_time_s': time.time(),
            'initial_joint_state': self.initial_joint_state,
            'observed_joint_states': self.observed_joint_states,
            'final_joint_state': self.joint_state,
            'detach_command_count': self.detach_commands,
            'detachable_joint_enabled': bool(
                self.get_parameter('expect_detachable_joint').value
            ),
            'verified_free': success,
            'object_pose_fresh': (
                time.monotonic() - self.object_pose_wall
                <= float(
                    self.get_parameter('pose_max_age_wall_s').value
                )
            ),
            'object_pose_m': self.object_pose,
            'object_motion_during_reset_m': self.motion_during_reset(),
            'ground_truth_source': (
                '/model/pick_object/pose (actual Gazebo model Pose_V)'
            ),
        }
        (self.output / 'initialization.json').write_text(
            json.dumps(payload, indent=2, allow_nan=False) + '\n',
            encoding='utf-8',
        )
        self.get_logger().info(json.dumps(payload, sort_keys=True))
        rclpy.shutdown()

    def tick(self):
        if self.finished:
            return
        now = time.monotonic()
        expect_joint = bool(
            self.get_parameter('expect_detachable_joint').value
        )
        if expect_joint and now - self.last_detach_wall >= 0.20:
            self.detach_pub.publish(Empty())
            self.detach_commands += 1
            self.last_detach_wall = now
        free_duration = (
            now - self.detached_since
            if self.detached_since is not None
            else (now - self.started_monotonic if not expect_joint else 0.0)
        )
        pose_fresh = (
            now - self.object_pose_wall
            <= float(self.get_parameter('pose_max_age_wall_s').value)
        )
        if reset_ready(
            expect_joint, self.joint_state, free_duration, pose_fresh
        ):
            reason = (
                'detached_feedback_and_fresh_pose'
                if expect_joint else 'detachable_joint_disabled_and_fresh_pose'
            )
            self.finish(True, reason)
            return
        if (
            now - self.started_monotonic
            > float(self.get_parameter('timeout_wall_s').value)
        ):
            self.finish(False, 'reset_verification_timeout')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
