import os
import launch
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, LogInfo, DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    # 🔹 Lecture des LaunchConfigurations
    namespace = launch.substitutions.LaunchConfiguration('namespace').perform(context)
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time').perform(context)
    autostart = launch.substitutions.LaunchConfiguration('autostart').perform(context)
    params_file = launch.substitutions.LaunchConfiguration('params_file').perform(context)
    bt_xml_file = launch.substitutions.LaunchConfiguration('bt_xml_file').perform(context)
    use_lifecycle_mgr = launch.substitutions.LaunchConfiguration('use_lifecycle_mgr').perform(context)
    map_subscribe_transient_local = launch.substitutions.LaunchConfiguration('map_subscribe_transient_local').perform(context)

    # 🔹 Répertoires
    bringup_dir = get_package_share_directory('limo_bringup')
    robot_pkg = get_package_share_directory('robot_exploration')

    # 🔹 Vérification du fichier paramètre
    if not params_file or not os.path.exists(params_file):
        raise RuntimeError(f"[NAV2] Invalid or missing params_file: {params_file}")

    # 🔹 Remappings
    remappings = [
        (f'{namespace}/tf', '/tf'),
        (f'{namespace}/tf_static', '/tf_static'),
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # 🔹 Substitutions dynamiques des paramètres
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local
    }

    # 🔹 Génération dynamique du fichier YAML configuré
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True,
    )

    # --- Débogage ---
    log_namespace = LogInfo(msg=[f"[NAV2] Namespace: {namespace}"])
    log_params = LogInfo(msg=[f"[NAV2] Params file: {params_file}"])

    # --- Définition des nœuds ---
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    lifecycle_manager = Node(
        condition=launch.conditions.IfCondition(use_lifecycle_mgr),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    # --- Actions retournées ---
    return [
        log_namespace,
        log_params,
        controller_server,
        planner_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager
    ]


def generate_launch_description():
    bringup_dir = get_package_share_directory('limo_bringup')
    robot_pkg = get_package_share_directory('robot_exploration')

    # 🔹 Déclaration des arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='limo1',
        description='Top-level namespace')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_pkg, 'param', 'git_nav2.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_bt_xml = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree XML file')

    declare_lifecycle_mgr = DeclareLaunchArgument(
        'use_lifecycle_mgr', default_value='true',
        description='Whether to launch the lifecycle manager')

    declare_transient_local = DeclareLaunchArgument(
        'map_subscribe_transient_local', default_value='false',
        description='Whether to set the map subscriber QoS to transient local')

    # 🔹 Description finale
    return launch.LaunchDescription([
        declare_namespace,
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_bt_xml,
        declare_lifecycle_mgr,
        declare_transient_local,
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()
