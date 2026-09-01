from glob import glob

from setuptools import setup


package_name = 'mobile_manipulator'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='dryanguasr',
    maintainer_email='dryanguasr@users.noreply.github.com',
    description='Reproducible ROS 2 Jazzy and Gazebo visual tracking example.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ball_detector=mobile_manipulator.ball_detector:main',
            'visual_tracker=mobile_manipulator.visual_tracker:main',
            'target_trajectory=mobile_manipulator.target_trajectory:main',
            'metrics_logger=mobile_manipulator.metrics_logger:main',
            'evidence_capture=mobile_manipulator.evidence_capture:main',
        ],
    },
)
