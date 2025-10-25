import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_with_namespace(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    if namespace not in ['limo1', 'limo2']:
        raise RuntimeError(f"Namespace invalide: {namespace}. Choisir 'limo1' ou 'limo2'.")

    pkg_robot = get_package_share_directory('robot_exploration')

    slam_config = os.path.join(pkg_robot, 'config', f'{namespace}_slam_config.yaml')

    nav2_params = os.path.join(pkg_robot, 'param', 'git_nav2.yaml')  # <- assure-toi que ce fichier existe
    bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )

    # Logs côté parent pour vérifier ce qu’on envoie
    log_parent = LogInfo(msg=[f"[BRINGUP] namespace={namespace}  params_file={nav2_params}  bt_xml={bt_xml}"])

    limo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot_start.launch.py')
        ),
        launch_arguments={'namespace': namespace}.items(),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_config
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'git_nav2.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'params_file': str(nav2_params),
            'use_sim_time': 'false'
        }.items(),
    )

    # id_srv = Node(
    #     package='robot_exploration',
    #     executable='identify_service',
    #     name='identify_robot_service',
    #     output='screen'
    # )
    # mission_action = Node(
    #     package='robot_exploration',
    #     executable='mission_server',
    #     name='mission_server',
    #     output='screen'
    # )

    group = GroupAction(actions=[
        PushRosNamespace(namespace),
        log_parent,
        limo_launch,
        # id_srv,
        # mission_action,
        slam_toolbox_launch,
        nav2_launch
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
