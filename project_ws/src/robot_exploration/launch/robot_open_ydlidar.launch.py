#!/usr/bin/python3
# Copyright 2020, EAIBOT
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
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
import launch
import launch_ros
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    # 🔹 Récupération dynamique des LaunchConfigurations
    namespace = launch.substitutions.LaunchConfiguration('namespace').perform(context)

    # 🔹 Construction dynamique des frames avec namespace
    base_frame = f"{namespace}/base_link"
    laser_frame = f"{namespace}/laser_frame"

    # 🔹 Définition du chemin du fichier de paramètres
    share_dir = get_package_share_directory('robot_exploration')
    params_file = os.path.join(share_dir, 'param', f'{namespace}_ydlidar.yaml')

    # 🔹 Liste des actions à lancer
    return [
        # LIDAR driver node (LifecycleNode)
        launch_ros.actions.LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[params_file]
        ),

        # TF statique base_link → laser_frame
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=[
                '0', '0', '0.02',  # translation x, y, z
                '0', '0', '0',     # rotation roll, pitch, yaw
                '1',               # quaternion w
                base_frame,
                laser_frame
            ]
        ),
    ]


def generate_launch_description():
    # 🔹 Déclaration des arguments
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value='limo1',
            description='Namespace obligatoire (ex: limo1 ou limo2)'
        ),
        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Chemin du fichier de paramètres YAML du LIDAR'
        ),
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()
