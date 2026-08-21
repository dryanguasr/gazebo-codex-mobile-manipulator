from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
 p=get_package_share_directory('mobile_manipulator'); x=os.path.join(p,'urdf','mobile_manipulator.urdf.xacro')
 gz=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')),launch_arguments={'gz_args':f'-r {os.path.join(p,"worlds","ball_arena.sdf")}'}.items())
 rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':open(x).read()}])
 spawn=Node(package='ros_gz_sim',executable='create',arguments=['-world','ball_arena','-name','mobile_manipulator','-file',x,'-x','0','-y','0','-z','0.25'])
 bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'])
 det=Node(package='mobile_manipulator',executable='ball_detector'); track=Node(package='mobile_manipulator',executable='visual_tracker')
 return LaunchDescription([gz,rsp,TimerAction(period=3.0,actions=[spawn,bridge,det,track])])
