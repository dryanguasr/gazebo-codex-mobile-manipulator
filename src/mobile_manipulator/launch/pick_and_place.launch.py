import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('mobile_manipulator')
    xacro = os.path.join(share, 'urdf', 'mobile_manipulator.urdf.xacro')
    world = os.path.join(share, 'worlds', 'pick_and_place.sdf')
    default_object_sdf = os.path.join(share, 'worlds', 'pick_object.sdf')
    config = os.path.join(share, 'config', 'pick_place_a1.yaml')
    output_dir = LaunchConfiguration('output_dir')
    run_id = LaunchConfiguration('run_id')
    source_sha = LaunchConfiguration('source_sha')
    source_dirty = LaunchConfiguration('source_dirty')
    seed = LaunchConfiguration('seed')
    capture_evidence = LaunchConfiguration('capture_evidence')
    evidence_fps = LaunchConfiguration('evidence_fps')
    attach_enabled = LaunchConfiguration('attach_enabled')
    grasp_mode = LaunchConfiguration('grasp_mode')
    spawn_object = LaunchConfiguration('spawn_object')
    object_sdf = LaunchConfiguration('object_sdf')
    object_x = LaunchConfiguration('object_x')
    object_y = LaunchConfiguration('object_y')
    object_z = LaunchConfiguration('object_z')
    gate_grasp_frame = LaunchConfiguration('gate_grasp_frame')
    action_name = LaunchConfiguration('action_name')
    negative_scenario = LaunchConfiguration('negative_scenario')

    managed_gazebo = os.path.join(
        get_package_prefix('mobile_manipulator'),
        'lib',
        'mobile_manipulator',
        'managed_gazebo',
    )
    gazebo = ExecuteProcess(
        cmd=[managed_gazebo, '--seed', seed, world],
        output='screen',
        sigterm_timeout='8',
        sigkill_timeout='4',
    )
    description = Command([
        'xacro ', xacro, ' manipulation_enabled:=true',
        ' pick_place_enabled:=', attach_enabled,
    ])
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(description, value_type=str),
            'use_sim_time': True,
        }],
    )
    robot_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'pick_and_place',
            '-name', 'mobile_manipulator',
            '-string', description,
            '-x', '0', '-y', '0', '-z', '0.02',
        ],
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/pick_object/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/model/pick_object/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/pick_place/evidence/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/pick_object/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/pick_object/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/pick_object/joint_state@std_msgs/msg/String[gz.msgs.StringMsg',
        ],
    )

    def controller_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '120'],
        )

    joint_state_spawner = controller_spawner('joint_state_broadcaster')
    base_spawner = controller_spawner('base_controller')
    arm_spawner = controller_spawner('arm_controller')
    gate = Node(
        package='mobile_manipulator',
        executable='pick_place_attach_gate',
        parameters=[
            config,
            {
                'attach_enabled': ParameterValue(attach_enabled, value_type=bool),
                'grasp_frame': gate_grasp_frame,
            },
        ],
        output='screen',
    )
    evaluator = Node(
        package='mobile_manipulator',
        executable='pick_place_evaluator',
        parameters=[
            config,
            {
                'output_dir': output_dir,
                'run_id': run_id,
                'source_sha': source_sha,
                'source_dirty': ParameterValue(source_dirty, value_type=bool),
                'grasp_mode': grasp_mode,
                'seed': ParameterValue(seed, value_type=int),
            },
        ],
        output='screen',
    )
    recorder = Node(
        package='mobile_manipulator',
        executable='pick_place_recorder',
        parameters=[{
            'output_dir': output_dir,
            'fps': ParameterValue(evidence_fps, value_type=float),
            'grasp_mode': grasp_mode,
        }],
        condition=IfCondition(capture_evidence),
        output='screen',
    )
    negative_injector = Node(
        package='mobile_manipulator',
        executable='pick_place_negative_injector',
        parameters=[{'scenario': negative_scenario}],
        condition=IfCondition(
            PythonExpression(["'", negative_scenario, "' != 'none'"])
        ),
        output='screen',
    )

    # Gazebo Sim 8 DetachableJoint attaches when its child appears. Spawn it,
    # issue RESET while that child exists, and only then start the measured
    # gate/evaluator/supervisor run.
    initializer = Node(
        package='mobile_manipulator',
        executable='pick_place_initializer',
        parameters=[{
            'output_dir': output_dir,
            'run_id': run_id,
            'source_sha': source_sha,
            'source_dirty': ParameterValue(source_dirty, value_type=bool),
            'seed': ParameterValue(seed, value_type=int),
            'expect_detachable_joint': ParameterValue(
                attach_enabled,
                value_type=bool,
            ),
        }],
        output='screen',
    )
    object_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'pick_and_place',
            '-name', 'pick_object',
            '-file', object_sdf,
            '-x', object_x, '-y', object_y, '-z', object_z,
        ],
        condition=IfCondition(spawn_object),
        output='screen',
    )
    supervisor = Node(
        package='mobile_manipulator',
        executable='pick_place_supervisor',
        parameters=[
            config,
            {
                'grasp_mode': grasp_mode,
                'action_name': action_name,
            },
        ],
        condition=IfCondition(spawn_object),
        output='screen',
    )
    supervisor_without_object = Node(
        package='mobile_manipulator',
        executable='pick_place_supervisor',
        parameters=[
            config,
            {
                'grasp_mode': grasp_mode,
                'action_name': action_name,
            },
        ],
        condition=UnlessCondition(spawn_object),
        output='screen',
    )
    stop_gazebo = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/server_control',
            '--reqtype', 'gz.msgs.ServerControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'stop: true',
        ],
        output='screen',
    )

    def reset_exit_actions(event, _context):
        if event.returncode == 0:
            return [TimerAction(
                period=0.5,
                actions=[
                    gate, evaluator, recorder, negative_injector, supervisor,
                ],
            )]
        return [stop_gazebo]

    after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_spawn,
            on_exit=[
                bridge, TimerAction(period=4.0, actions=[joint_state_spawner]),
                TimerAction(period=2.0, actions=[object_spawn]),
            ],
        )
    )
    after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[base_spawner],
        )
    )
    after_base = RegisterEventHandler(
        OnProcessExit(
            target_action=base_spawner,
            on_exit=[arm_spawner],
        )
    )
    after_object = RegisterEventHandler(
        OnProcessExit(
            target_action=object_spawn,
            on_exit=[TimerAction(period=1.0, actions=[initializer])],
        )
    )
    after_reset = RegisterEventHandler(
        OnProcessExit(
            target_action=initializer,
            on_exit=reset_exit_actions,
        )
    )
    without_object_start = TimerAction(
        period=3.5,
        actions=[gate, evaluator, recorder, negative_injector],
        condition=UnlessCondition(spawn_object),
    )

    after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_spawner,
            on_exit=[TimerAction(
                period=1.5,
                actions=[supervisor_without_object],
            )],
        )
    )

    stop_after_supervisor = RegisterEventHandler(
        OnProcessExit(
            target_action=supervisor,
            on_exit=[stop_gazebo],
        )
    )
    stop_after_supervisor_without_object = RegisterEventHandler(
        OnProcessExit(
            target_action=supervisor_without_object,
            on_exit=[stop_gazebo],
        )
    )

    shutdown_after_gazebo = RegisterEventHandler(
        OnProcessExit(
            target_action=stop_gazebo,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[EmitEvent(event=Shutdown(reason='A1 terminal'))],
                )
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/pick_place_a1'),
        DeclareLaunchArgument('run_id', default_value='manual'),
        DeclareLaunchArgument('source_sha', default_value='unknown'),
        DeclareLaunchArgument('source_dirty', default_value='false'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('capture_evidence', default_value='false'),
        DeclareLaunchArgument('evidence_fps', default_value='60.0'),
        DeclareLaunchArgument('attach_enabled', default_value='true'),
        DeclareLaunchArgument('grasp_mode', default_value='attach_conditioned'),
        DeclareLaunchArgument('spawn_object', default_value='true'),
        DeclareLaunchArgument('object_sdf', default_value=default_object_sdf),
        DeclareLaunchArgument('object_x', default_value='-0.044'),
        DeclareLaunchArgument('object_y', default_value='-0.225'),
        DeclareLaunchArgument('object_z', default_value='0.1625'),
        DeclareLaunchArgument('gate_grasp_frame', default_value='poppy_grasp_frame'),
        DeclareLaunchArgument(
            'action_name',
            default_value='/arm_controller/follow_joint_trajectory',
        ),
        DeclareLaunchArgument('negative_scenario', default_value='none'),
        gazebo,
        state_publisher,
        robot_spawn,
        after_robot,
        after_joint_state,
        after_base,
        after_arm,
        after_reset,
        after_object,
        without_object_start,
        stop_after_supervisor,
        stop_after_supervisor_without_object,
        shutdown_after_gazebo,
    ])
