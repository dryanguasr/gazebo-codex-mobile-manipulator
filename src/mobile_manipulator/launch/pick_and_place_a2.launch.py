"""Bounded physical-contact A2 launch with DetachableJoint disabled."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base_launch = os.path.join(
        get_package_share_directory('mobile_manipulator'),
        'launch',
        'pick_and_place.launch.py',
    )
    forwarded = {
        name: LaunchConfiguration(name)
        for name in (
            'output_dir',
            'run_id',
            'source_sha',
            'source_dirty',
            'seed',
            'capture_evidence',
        )
    }
    forwarded.update({
        'attach_enabled': 'false',
        'grasp_mode': 'physical_contact',
    })
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/pick_place_a2'),
        DeclareLaunchArgument('run_id', default_value='a2_bounded'),
        DeclareLaunchArgument('source_sha', default_value='unknown'),
        DeclareLaunchArgument('source_dirty', default_value='false'),
        DeclareLaunchArgument('seed', default_value='201'),
        DeclareLaunchArgument('capture_evidence', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments=forwarded.items(),
        ),
    ])
