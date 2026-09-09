"""Complete mobile exercise: contact-gated rigid grasp and separated stations."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('mobile_manipulator')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'pick_and_place.launch.py')
            ),
            launch_arguments={
                'world_file': os.path.join(share, 'worlds', 'pick_and_place_mobile.sdf'),
                'config_file': os.path.join(share, 'config', 'pick_place_mobile.yaml'),
                'object_sdf': os.path.join(share, 'worlds', 'pick_object_mobile.sdf'),
                'mobile_transport': 'true',
                'attach_enabled': 'true',
                'grasp_mode': 'attach_conditioned',
            }.items(),
        ),
    ])
