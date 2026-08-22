from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('mobile_manipulator')
    xacro_file = os.path.join(package_share, 'urdf', 'mobile_manipulator.urdf.xacro')
    urdf_file = '/tmp/mobile_manipulator.urdf'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s {os.path.join(package_share, "worlds", "ball_arena.sdf")}'}.items())
    expand = ExecuteProcess(cmd=['xacro', xacro_file, '-o', urdf_file])
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', xacro_file])}])
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'ball_arena', '-name', 'mobile_manipulator', '-file', urdf_file,
                   '-x', '0', '-y', '0', '-z', '0.25'])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'])
    # Each spawner waits for controller_manager; no wall-clock launch delay is used.
    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[controller, '--controller-manager-timeout', '120'])
        for controller in ['base_controller', 'arm_controller']]
    app_nodes = [
        bridge,
        Node(package='mobile_manipulator', executable='ball_detector'),
        Node(package='mobile_manipulator', executable='visual_tracker')]
    return LaunchDescription([
        gazebo, expand, state_publisher,
        RegisterEventHandler(OnProcessExit(target_action=expand, on_exit=[spawn])),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawners + app_nodes))])
