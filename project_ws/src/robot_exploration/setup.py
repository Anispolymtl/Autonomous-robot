import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'param'), glob('param/*')),
    ],
    install_requires=['setuptools', 'limo_interfaces'],
    zip_safe=True,
    maintainer='scoobyfelix', 
    maintainer_email='felix.paille-dowell@polymtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'identify_service = robot_exploration.service.identify_robot_service:main',
        'mission_server = robot_exploration.mission.mission_server:main',
        'mission_real = robot_exploration.mission.mission_server_real:main',
        'explorer_node = robot_exploration.autonomous_exploration.autonomous_exploration:main',
        'map_merge = robot_exploration.map_merge.map_merge:main',
        'imu_frame_remapper = robot_exploration.tf_remapping.imu_frame_remapper:main',
        'base_manager = robot_exploration.localization.base_manager:main',
        'initial_pose_publisher = robot_exploration.localization.initial_pose_publisher:main',
        'robot_pose_monitor = robot_exploration.localization.robot_pose_monitor:main',

    ],
    },
)