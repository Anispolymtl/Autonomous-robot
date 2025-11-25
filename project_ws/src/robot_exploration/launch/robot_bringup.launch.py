import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_with_namespace(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace not in ['limo1', 'limo2']:
        raise RuntimeError(f"Namespace invalide: {namespace}. Choisir 'limo1' ou 'limo2'.")

    pkg_robot = get_package_share_directory('robot_exploration')
    explore_pkg = get_package_share_directory('explore_lite')

    # slam_config = os.path.join(pkg_robot, 'config', f'{namespace}_slam_config.yaml')

    cartographer_file = f'{namespace}_cartographer_2d.lua'

    nav2_params = os.path.join(pkg_robot, 'param', f'{namespace}_nav.yaml')


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

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot_localization.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'params_file': nav2_params,
            'use_sim_time': 'false'
        }.items(),
    )
    robot_position_monitor = Node(
            package="robot_exploration",
            executable="robot_position_monitor",
            name="robot_position_monitor",
            output="screen",
            parameters=[{"rate": 10.0}]
    )

    id_srv = Node(
        package='robot_exploration',
        executable='identify_service',
        name='identify_robot_service',
        output='screen'
    )

    base_srv = Node(
        package='robot_exploration',
        executable='base_manager',
        name='base_manager',
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

    group = GroupAction(actions=[
        PushRosNamespace(namespace),
        limo_launch,
        robot_position_monitor,
        id_srv,
        base_srv,
        mission_action,
        cartographer_launch,
        nav2_launch,
        explore_launch
    ])

    return [group]

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
