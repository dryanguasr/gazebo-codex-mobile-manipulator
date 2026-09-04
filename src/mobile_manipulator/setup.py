from glob import glob
from pathlib import Path

from setuptools import setup


package_name = 'mobile_manipulator'

asset_root = Path('meshes/poppy_ergo_jr')
runtime_mesh_data_files = []
for category, pattern in (
    ('visual', '*.stl'),
    ('collision', '*.stl'),
    ('official', '*.dae'),
):
    directory = asset_root / category
    runtime_mesh_data_files.append(
        (
            'share/' + package_name + '/' + directory.as_posix(),
            [str(path) for path in sorted(directory.glob(pattern))],
        )
    )

# Source STEP/STL files stay in Git for reproducibility; Gazebo needs only runtime meshes.
runtime_manifest_files = [
    str(asset_root / 'asset_manifest.json'),
]
hardware_license_files = [
    str(asset_root / 'source/hardware/LICENSE.md'),
    str(asset_root / 'source/hardware/README.md'),
]
official_license_files = [
    str(asset_root / 'official/README.md'),
    str(asset_root / 'official/LICENSE_GPL-3.0.txt'),
]

setup(
    name=package_name,
    version='0.4.0',
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
        (
            'share/' + package_name + '/meshes/poppy_ergo_jr',
            runtime_manifest_files,
        ),
        (
            'share/' + package_name + '/meshes/poppy_ergo_jr/licenses/hardware',
            hardware_license_files,
        ),
        (
            'share/' + package_name + '/meshes/poppy_ergo_jr/licenses/official',
            official_license_files,
        ),
        *runtime_mesh_data_files,
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='dryanguasr',
    maintainer_email='dryanguasr@users.noreply.github.com',
    description=(
        'ROS 2 mobile manipulator with a reproducible CAD pipeline and '
        'mechanically consolidated Poppy Ergo Jr arm.'
    ),
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
