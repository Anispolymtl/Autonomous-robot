# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    robot_pkg = get_package_share_directory('robot_exploration')
    params_file = LaunchConfiguration('params_file')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_pkg, 'param', 'limo1_ekf.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    efk_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/odometry/filtered', 'odom/filtered'),
            ('/odometry', 'odom'),
            ('/imu/data', 'imu/data')
        ]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        efk_node
])