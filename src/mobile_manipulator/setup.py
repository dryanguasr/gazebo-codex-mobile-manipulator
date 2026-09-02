from glob import glob
from pathlib import Path

from setuptools import setup


package_name = 'mobile_manipulator'
mesh_data_files = []
mesh_root = Path('meshes')
for directory in sorted(
    {path.parent for path in mesh_root.rglob('*') if path.is_file()}
):
    mesh_data_files.append(
        (
            'share/' + package_name + '/' + directory.as_posix(),
            [str(path) for path in sorted(directory.iterdir()) if path.is_file()],
        )
    )

setup(
    name=package_name,
    version='0.3.0',
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
        *mesh_data_files,
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='dryanguasr',
    maintainer_email='dryanguasr@users.noreply.github.com',
    description='ROS 2 mobile manipulator with a CAD-derived Poppy Ergo Jr arm.',
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
