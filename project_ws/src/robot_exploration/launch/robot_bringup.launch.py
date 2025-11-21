import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def launch_with_namespace(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace not in ['limo1', 'limo2']:
        raise RuntimeError(f"Namespace invalide: {namespace}. Choisir 'limo1' ou 'limo2'.")

    pkg_robot = get_package_share_directory('robot_exploration')
    explore_pkg = get_package_share_directory('explore_lite')
    merge_pkg = get_package_share_directory('multirobot_map_merge')

    # slam_config = os.path.join(pkg_robot, 'config', f'{namespace}_slam_config.yaml')

    cartographer_file = f'{namespace}_cartographer_2d.lua'

    nav2_params = os.path.join(pkg_robot, 'param', f'{namespace}_nav.yaml')
    
    # zenoh_config = os.path.join(pkg_robot, 'config', 'zenoh_peer.json5')
    
    # zenoh_bridge = ExecuteProcess(
    #     cmd=[
    #         "zenoh-bridge-ros2dds",
    #         "-c",
    #         zenoh_config],
    #     name="zenoh_bridge",
    #     output="screen",
    #     shell=False
    # )

    only_if_limo1 = IfCondition(
        PythonExpression(['"', LaunchConfiguration("namespace"), '" == "limo1"'])
    )

    limo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot_start.launch.py')
        ),
        launch_arguments={'namespace': namespace}.items(),
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot_cartographer.launch.py')
        ),
        launch_arguments={
            'configuration_basename': cartographer_file
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot_navigation.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'params_file': nav2_params,
            'use_sim_time': 'false'
        }.items(),
    )

    id_srv = Node(
        package='robot_exploration',
        executable='identify_service',
        name='identify_robot_service',
        output='screen'
    )

    mission_action = Node(
        package='robot_exploration',
        executable='mission_server',
        name='mission_server',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(explore_pkg, 'launch', 'explore.launch.py')
        ))

    map_merge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(merge_pkg, 'launch', 'map_merge.launch.py')
        )
    )

    group = GroupAction(actions=[
        PushRosNamespace(namespace),
        limo_launch,
        id_srv,
        mission_action,
        cartographer_launch,
        nav2_launch,
        explore_launch
    ])

    return [group,
            map_merge]

def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='limo1',
        description="Namespace obligatoire (limo1 ou limo2)"
    )

    return LaunchDescription([
        declare_namespace,
        OpaqueFunction(function=launch_with_namespace)
    ])
