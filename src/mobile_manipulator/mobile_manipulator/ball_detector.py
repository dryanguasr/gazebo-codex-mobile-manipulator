import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
class Detector(Node):
 def __init__(self):
  super().__init__('ball_detector'); self.b= CvBridge(); self.create_subscription(Image,'/camera/image_raw',self.cb,10); self.pub=self.create_publisher(Vector3,'/ball/measurement',10); self.dbg=self.create_publisher(Image,'/ball/debug',10)
  self.radius=0.12; self.fx=320.0
 def cb(self,msg):
  im=self.b.imgmsg_to_cv2(msg,'bgr8'); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
  m=cv2.inRange(hsv,np.array([0,120,80]),np.array([12,255,255]))|cv2.inRange(hsv,np.array([170,120,80]),np.array([180,255,255]))
  cs,_=cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE); out=im.copy(); z=Vector3()
  if cs:
   c=max(cs,key=cv2.contourArea); (x,y),r=cv2.minEnclosingCircle(c)
   if r>4: z.x=(x-im.shape[1]/2)/(im.shape[1]/2); z.y=(y-im.shape[0]/2)/(im.shape[0]/2); z.z=self.fx*2*self.radius/(2*r); cv2.circle(out,(int(x),int(y)),int(r),(0,255,0),2); cv2.putText(out,f'd={z.z:.2f}m',(10,25),0,0.6,(0,255,0),2)
  self.pub.publish(z); self.dbg.publish(self.b.cv2_to_imgmsg(out,'bgr8'))
def main(): rclpy.init(); rclpy.spin(Detector()); rclpy.shutdown()
