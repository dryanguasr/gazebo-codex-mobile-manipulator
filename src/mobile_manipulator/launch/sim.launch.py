import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('mobile_manipulator')
    xacro_file = os.path.join(
        package_share, 'urdf', 'mobile_manipulator.urdf.xacro'
    )
    world_file = os.path.join(package_share, 'worlds', 'ball_arena.sdf')

    tracking_enabled = LaunchConfiguration('tracking_enabled')
    target_mode = LaunchConfiguration('target_mode')
    metrics_enabled = LaunchConfiguration('metrics_enabled')
    metrics_output_dir = LaunchConfiguration('metrics_output_dir')
    run_label = LaunchConfiguration('run_label')
    duration_s = LaunchConfiguration('duration_s')
    target_distance_m = LaunchConfiguration('target_distance_m')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r -s {world_file}'}.items(),
    )
    xacro_command = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(xacro_command, value_type=str)
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ],
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world',
            'ball_arena',
            '-name',
            'mobile_manipulator',
            '-string',
            xacro_command,
            '-x',
            '0',
            '-y',
            '0',
            '-z',
            '0.02',
        ],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/world/ball_arena/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
    )
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller,
                '--controller-manager-timeout',
                '120',
            ],
        )
        for controller in [
            'joint_state_broadcaster',
            'base_controller',
            'arm_controller',
        ]
    ]
    detector = Node(
        package='mobile_manipulator',
        executable='ball_detector',
        parameters=[{'use_sim_time': True}],
    )
    tracker = Node(
        package='mobile_manipulator',
        executable='visual_tracker',
        condition=IfCondition(tracking_enabled),
        parameters=[
            {
                'use_sim_time': True,
                'target_distance_m': ParameterValue(
                    target_distance_m, value_type=float
                ),
            }
        ],
    )
    target = Node(
        package='mobile_manipulator',
        executable='target_trajectory',
        parameters=[{'use_sim_time': True, 'mode': target_mode}],
    )
    metrics = Node(
        package='mobile_manipulator',
        executable='metrics_logger',
        condition=IfCondition(metrics_enabled),
        parameters=[
            {
                'use_sim_time': True,
                'output_dir': metrics_output_dir,
                'run_label': run_label,
                'duration_s': ParameterValue(duration_s, value_type=float),
                'target_distance_m': ParameterValue(
                    target_distance_m, value_type=float
                ),
            }
        ],
    )

    start_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[bridge, *spawners, detector, tracker, target, metrics],
        )
    )
    stop_after_metrics = RegisterEventHandler(
        OnProcessExit(
            target_action=metrics,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason='metrics collection completed')
                )
            ],
        )
    )
    stop_gazebo_server = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        'pkill',
                        '-f',
                        '-KILL',
                        f'gz sim.*{world_file}',
                    ]
                )
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('tracking_enabled', default_value='true'),
            DeclareLaunchArgument('target_mode', default_value='static'),
            DeclareLaunchArgument('metrics_enabled', default_value='false'),
            DeclareLaunchArgument(
                'metrics_output_dir',
                default_value='/tmp/mobile_manipulator_metrics',
            ),
            DeclareLaunchArgument('run_label', default_value='run'),
            DeclareLaunchArgument('duration_s', default_value='30.0'),
            DeclareLaunchArgument('target_distance_m', default_value='1.2'),
            gazebo,
            state_publisher,
            spawn,
            start_after_spawn,
            stop_after_metrics,
            stop_gazebo_server,
        ]
    )
