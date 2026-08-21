import rclpy
from rclpy.node import Node
class Metrics(Node):
 def __init__(self): super().__init__('metrics_logger')
def main(): rclpy.init(); rclpy.spin(Metrics()); rclpy.shutdown()
