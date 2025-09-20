import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_ros_limo = get_package_share_directory('limo_bringup')

    # Lancer les drivers du robot
    limo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_limo, 'launch', 'limo_start.launch.py'))
    )

    # Lancer le service d'identification
    srv_id = Node(
        package='robot_exploration',
        executable='identify_service',
        name='identify_robot_service',
        output='screen'
    )

    return LaunchDescription([
        limo_launch,
        srv_id
    ])