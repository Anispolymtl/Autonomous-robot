# Copyright 2022 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_example_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_example_gazebo')
    pkg_project_description = get_package_share_directory('ros_gz_example_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    limo_cfg = os.path.join(pkg_project_bringup, 'config', 'limo_params.yaml')

    # Load the SDF file from "description" package
    sdf_file_limo1 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive1', 'model.sdf')
    with open(sdf_file_limo1, 'r') as infp:
        robot_desc_limo1 = infp.read()

    sdf_file_limo2 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive2', 'model.sdf')
    with open(sdf_file_limo2, 'r') as infp:
        robot_desc_limo2 = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher_limo1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_limo1',
        namespace='limo1',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc_limo1},
        ]
    )

    robot_state_publisher_limo2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_limo2',
        namespace='limo2',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc_limo2},
        ]
    )

    # Visualize in RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'limo_diff_drive.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_example_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    srv1_id = Node(
        package='robot_exploration',
        executable='identify_service',
        name='identify_service',
        namespace='limo1',
        output='both'
    )

    srv2_id = Node(
        package='robot_exploration',
        executable='identify_service',
        name='identify_service',
        namespace='limo2',
        output='both'
    )

    mission_action_1 = Node(
        package='robot_exploration',
        executable='mission_server',
        name='mission_server',
        namespace='limo1',
        output='screen',
    )

    mission_action_2 = Node(
        package='robot_exploration',
        executable='mission_server',
        name='mission_server',
        namespace='limo2',
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='66'),
        gz_sim,
        # DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        robot_state_publisher_limo1,
        robot_state_publisher_limo2,
        # rviz
        srv1_id,
        srv2_id,
        mission_action_1,
        mission_action_2
    ])
