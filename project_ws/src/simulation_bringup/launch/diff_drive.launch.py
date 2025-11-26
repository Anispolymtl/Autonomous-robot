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
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable,GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import Command

from launch_ros.actions import Node

def generate_launch_description():

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('simulation_bringup')
    pkg_project_gazebo = get_package_share_directory('simulation_gazebo')
    pkg_project_description = get_package_share_directory('simulation_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    explore_pkg = get_package_share_directory('explore_lite')


    limo1_slam_sim_config = os.path.join(get_package_share_directory("simulation_bringup"),
                                   'config', 'limo1_slam_sim_config.yaml')
    limo2_slam_sim_config = os.path.join(get_package_share_directory("simulation_bringup"),
                                   'config', 'limo2_slam_sim_config.yaml')

    explore_limo1_params = os.path.join(
        explore_pkg, 'config', 'limo1_params_sim.yaml'
    )
    explore_limo2_params = os.path.join(
        explore_pkg, 'config', 'limo2_params_sim.yaml'
    )

    # Load the SDF file from "description" package
    sdf_file_limo1 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive1', 'model.sdf')
    with open(sdf_file_limo1, 'r') as infp:
        robot_desc_limo1 = infp.read()

    sdf_file_limo2 = os.path.join(pkg_project_description, 'models', 'limo_diff_drive2', 'model.sdf')
    with open(sdf_file_limo2, 'r') as infp:
        robot_desc_limo2 = infp.read()
    


    # Setup to launch the simulator and Gazebo world
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': PathJoinSubstitution([
    #         pkg_project_gazebo,
    #         'worlds',
    #         'diff_drive.sdf'
    #     ])}.items(),
    # )
    
    world_file = Command(["ros2 ", "run ", "simulation_bringup ", "spawn_random.py"])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim,'launch','gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file}.items(),
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
            'config_file': os.path.join(pkg_project_bringup, 'config', 'simulation_bridge.yaml'),
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

    slam_1 = GroupAction([
        PushRosNamespace('limo1'),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("robot_exploration"),
                        "launch",
                        "robot_slam.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam_params_file" : limo1_slam_sim_config,
                    "namespace": "limo1",
                }.items(),
            )
    ])

    slam_2 = GroupAction([
        PushRosNamespace('limo2'),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("robot_exploration"),
                        "launch",
                        "robot_slam.launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "slam_params_file" : limo2_slam_sim_config,
                    "namespace":"limo2"
                }.items(),
            )
    ])


    merge_map_node = Node(
        package='robot_exploration',
        executable='map_merge',
        name='merge_map_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ("/map1", "/limo1/map"),
            ("/map2", "/limo2/map"),
            ("/merge_map", "/merged_map"),   # topic fusionné
        ],
    )

    static_tf_limo1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_limo1',
        arguments=['0', '0', '0', '0', '0', '0', 'merge_map', 'limo1/map']
    )

    static_tf_limo2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_limo2',
        arguments=['0', '0', '0', '0', '0', '0', 'merge_map', 'limo2/map']
    )

    static_merge_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_merge_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'merge_map']
    )
    
    nav2_1 = GroupAction([
        PushRosNamespace('limo1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("robot_exploration"),
                    "launch",
                    "robot_navigation.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": os.path.join(
                    get_package_share_directory("simulation_bringup"),
                    "param",
                    "limo1_nav_sim.yaml"
                ),
                "namespace": "limo1",
            }.items(),
        )
    ])

    nav2_2 = GroupAction([
        PushRosNamespace('limo2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("robot_exploration"),
                    "launch",
                    "robot_navigation.launch.py",
                )
            ),
            launch_arguments={
                "use_sim_time": "true",
                "params_file": os.path.join(
                    get_package_share_directory("simulation_bringup"),
                    "param",
                    "limo2_nav_sim.yaml"
                ),
                "namespace": "limo2",
            }.items(),
        )
    ])

    explorer_1 = Node(
        package='robot_exploration',
        executable='explorer_node',
        name='explorer_node',
        namespace='limo1',
        output='screen',
    )

    explorer_2 = Node(
        package='robot_exploration',
        executable='explorer_node',
        name='explorer_node',
        namespace='limo2',
        output='screen',
    )

    battery_limo1 = Node(
    package='robot_exploration',
    executable='battery_manager',
    name='battery_manager',
    namespace='limo1',
    output='screen'
    )

    base_limo1 = Node(
        package='robot_exploration',
        executable='base_manager',
        name='base_manager',
        namespace='limo1',
        output='screen'
    )

    base_limo2 = Node(
        package='robot_exploration',
        executable='base_manager',
        name='base_manager',
        namespace='limo2',
        output='screen'
    )

    robot_position_monitor_limo1 = GroupAction([
        PushRosNamespace('limo1'),
        Node(
            package="robot_exploration",
            executable="robot_position_monitor",
            name="robot_position_monitor",
            output="screen",
            parameters=[{"rate": 10.0}]
        )
    ])

    robot_position_monitor_limo2 = GroupAction([
        PushRosNamespace('limo2'),
        Node(
            package="robot_exploration",
            executable="robot_position_monitor",
            name="robot_position_monitor",
            output="screen",
            parameters=[{"rate": 10.0}]
        )
    ])

    explore_limo1 = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace='limo1',
        parameters=[
            explore_limo1_params,       # <-- ICI on charge le YAML
            {'use_sim_time': True}
        ],
        output='screen'
    )

    explore_limo2 = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        namespace='limo2',
        parameters=[
            explore_limo2_params,       # <-- ICI aussi
            {'use_sim_time': True}
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='66'),
        gz_sim,
        # DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        robot_state_publisher_limo1,
        robot_state_publisher_limo2,
        # rviz

        # Identification
        srv1_id,
        srv2_id,

        # Mission managment
        mission_action_1,
        mission_action_2,

        # Slam toolbox
        slam_1,
        slam_2,

        # Map Merge
        merge_map_node,
        static_tf_limo1,
        static_tf_limo2,
        static_merge_map_tf,

        #navigation2
        nav2_1,
        nav2_2,

        # Exploration
        # explorer_1,
        # explorer_2,
        explore_limo1,
        explore_limo2,

        # battery_limo1,
        base_limo1,
        base_limo2,
        robot_position_monitor_limo1, 
        robot_position_monitor_limo2

    ])