import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    pkg_explore = get_package_share_directory('explore_lite')

    config_path = os.path.join(pkg_explore, 'config', f'{namespace}_params.yaml')
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Le fichier de configuration '{config_path}' est introuvable !")
    
    return [
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            parameters=[ParameterFile(config_path, allow_substs=True),
                        {'use_sim_time': False}],
            output='screen',
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        )
    ]


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description="Namespace obligatoire (limo1 ou limo2)"
    )

    return LaunchDescription([
        declare_namespace,
        OpaqueFunction(function=launch_setup)
    ])
