"""Inject only declared negative-test faults into pick-and-place runs."""

import json

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PickPlaceNegativeInjector(Node):
    def __init__(self):
        super().__init__('pick_place_negative_injector')
        self.declare_parameter('scenario', 'none')
        self.scenario = str(self.get_parameter('scenario').value)
        self.state = 'IDLE'
        self.cancel_sent = False
        self.navigation_started_ns = None
        self.base_pub = self.create_publisher(
            TwistStamped, '/base_controller/cmd_vel', 20
        )
        self.cancel_pub = self.create_publisher(
            Bool, '/pick_place/cancel', 10
        )
        self.create_subscription(
            String, '/pick_place/status', self.status_callback, 50
        )
        self.create_timer(
            0.02,
            self.tick,
            clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self.get_logger().warning(
            f'Negative-test injector active: {self.scenario}'
        )

    def status_callback(self, message):
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.state = str(data.get('state', self.state))
        if self.state == 'NAVIGATE' and self.navigation_started_ns is None:
            self.navigation_started_ns = self.get_clock().now().nanoseconds
        if (
            self.scenario == 'cancel_during_transport'
            and data.get('event') == 'transition'
            and data.get('to_state') == 'TRANSFER'
            and not self.cancel_sent
        ):
            cancel = Bool()
            cancel.data = True
            self.cancel_pub.publish(cancel)
            self.cancel_sent = True
            self.get_logger().warning('Injected transport cancellation')

    def tick(self):
        if (
            self.scenario == 'cancel_mobile_route'
            and self.state == 'NAVIGATE' and self.navigation_started_ns is not None
            and not self.cancel_sent
            and (self.get_clock().now().nanoseconds - self.navigation_started_ns) / 1e9 > 3.0
        ):
            self.cancel_pub.publish(Bool(data=True))
            self.cancel_sent = True
            self.get_logger().warning('Injected cancellation while driving with load')
        if self.scenario != 'base_motion':
            return
        if self.state in (
            'OPEN', 'PREGRASP', 'APPROACH', 'CLOSE', 'VERIFY_GRASP',
            'LIFT', 'HOLD', 'TRANSFER', 'LOWER',
        ):
            command = TwistStamped()
            command.header.stamp = self.get_clock().now().to_msg()
            command.header.frame_id = 'base_footprint'
            command.twist.linear.x = 0.20
            self.base_pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNegativeInjector()
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
