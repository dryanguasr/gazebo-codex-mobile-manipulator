from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
 p=get_package_share_directory('mobile_manipulator'); x=os.path.join(p,'urdf','mobile_manipulator.urdf.xacro'); u='/tmp/mobile_manipulator.urdf'
 gz=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py')),launch_arguments={'gz_args':f'-r {os.path.join(p,"worlds","ball_arena.sdf")}'}.items())
 expand=ExecuteProcess(cmd=['xacro',x,'-o',u])
 rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':Command(['xacro ',x])}])
 spawn=Node(package='ros_gz_sim',executable='create',arguments=['-world','ball_arena','-name','mobile_manipulator','-file',u,'-x','0','-y','0','-z','0.25'])
 bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'])
 return LaunchDescription([gz,expand,rsp,TimerAction(period=3.0,actions=[spawn,bridge,Node(package='mobile_manipulator',executable='ball_detector'),Node(package='mobile_manipulator',executable='visual_tracker')])])
