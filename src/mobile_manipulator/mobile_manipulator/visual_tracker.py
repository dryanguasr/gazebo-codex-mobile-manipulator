import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist
class Tracker(Node):
 def __init__(self):
  super().__init__('visual_tracker'); self.create_subscription(Vector3,'/ball/measurement',self.cb,10); self.pub=self.create_publisher(Twist,'/base_controller/cmd_vel_unstamped',10); self.target=1.0
 def cb(self,m):
  t=Twist()
  if m.z>0.05:
   t.angular.z=max(-1.2,min(1.2,-1.4*m.x))
   t.linear.x=max(-0.45,min(0.45,0.7*(m.z-self.target)))*(1-min(0.8,abs(m.x)))
  self.pub.publish(t)
def main(): rclpy.init(); rclpy.spin(Tracker()); rclpy.shutdown()
