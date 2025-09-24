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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scoobyfelix',
    maintainer_email='felix.paille-dowell@polymtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'client_service = robot_exploration.service.client_service_test_node:main',
        'identify_service = robot_exploration.service.identify_robot_service:main',
        #'test_node = robot_exploration.test_node:main',
    ],
    },
)