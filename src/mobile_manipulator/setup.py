from glob import glob
from setuptools import setup
name = 'mobile_manipulator'
setup(name=name, version='0.1.0', packages=[name],
 data_files=[('share/ament_index/resource_index/packages',['resource/'+name]),
 ('share/'+name,['package.xml']),('share/'+name+'/urdf',glob('urdf/*')),
 ('share/'+name+'/config',glob('config/*')),('share/'+name+'/worlds',glob('worlds/*')),
 ('share/'+name+'/launch',glob('launch/*'))], install_requires=['setuptools'], zip_safe=True,
 maintainer='Codex Local', maintainer_email='codex@local.invalid', license='Apache-2.0',
 entry_points={'console_scripts':['ball_detector=mobile_manipulator.ball_detector:main','visual_tracker=mobile_manipulator.visual_tracker:main','target_trajectory=mobile_manipulator.target_trajectory:main','metrics_logger=mobile_manipulator.metrics_logger:main']})
