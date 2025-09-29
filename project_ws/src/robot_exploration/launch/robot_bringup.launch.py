import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_limo(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace not in ['limo1', 'limo2']:
        raise RuntimeError(f"Namespace invalide: {namespace}. Choisir 'limo1' ou 'limo2'.")

    pkg_limo_bringup = get_package_share_directory('robot_exploration')

    limo_launch = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_limo_bringup, 'launch', 'robot_start.launch.py')
                )
            ),
            Node(
                package='robot_exploration',
                executable='identify_service',
                name='identify_robot_service',
                output='screen'
            ),
            Node(
                package='robot_exploration',
                executable='mission_server',
                name='mission_server',
                output='screen'
            )
        ]
    )
    return [limo_launch]

def launch_sim(context, *args, **kwargs):
    pkg_sim_bringup = get_package_share_directory('simulation_bringup')
    SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='66'),

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim_bringup, 'launch', 'diff_drive.launch.py'))
    )

    return [sim_launch]

def generate_launch_description():
    use_limo = LaunchConfiguration('use_limo')
    namespace = LaunchConfiguration('namespace')

    declare_use_limo = DeclareLaunchArgument(
        'use_limo',
        default_value='false',
        description='Inclure limo_bringup si true'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='limo1',
        description="Namespace obligatoire (limo1 ou limo2) si use_limo est vrai"
    )

    # Lancer le service d'identification

    return LaunchDescription([
        declare_use_limo,
        declare_namespace,
        OpaqueFunction(function=launch_limo, condition=IfCondition(use_limo)),
        OpaqueFunction(function=launch_sim, condition=UnlessCondition(use_limo))
    ])
