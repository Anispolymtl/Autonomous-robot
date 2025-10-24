import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction

def launch_setup(context, *args, **kwargs):
    namespace = launch.substitutions.LaunchConfiguration('namespace').perform(context)
    port_name = launch.substitutions.LaunchConfiguration('port_name').perform(context)
    # odom_topic_name = launch.substitutions.LaunchConfiguration('odom_topic_name').perform(context)
    open_rviz = launch.substitutions.LaunchConfiguration('open_rviz').perform(context)

    # 🔹 Construction des frames avec namespace évalué
    odom_frame = f"{namespace}/odom"
    odom_topic_name = f"{namespace}/odom"
    base_frame = f"{namespace}/base_link"
    imu_frame = f"{namespace}/imu_link"

    # 🔹 Liste des actions de lancement
    return [

        # RViz (optionnel)
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            on_exit=launch.actions.Shutdown(),
            condition=launch.conditions.IfCondition(open_rviz)
        ),

        # 🔹 Transformation statique base_link → imu_link
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', f"{base_frame}", f"{imu_frame}"]
        ),

        # launch_ros.actions.Node(
        #     package='robot_pose_ekf',
        #     executable='robot_pose_ekf',
        #     name='robot_pose_ekf',
        #     parameters=[
        #         {
        #             'output_frame': 'odom'
        #         },
        #         {
        #             'base_footprint_frame': 'base_link'
        #         }
        #     ]
        # ),

        # 🔹 robot_base.launch.py (Limo base node)
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_exploration'),
                             'launch', 'robot_base.launch.py')
            ),
            launch_arguments={
                'port_name': port_name,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'odom_topic_name': odom_topic_name
            }.items()
        ),

        # 🔹 LIDAR
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_exploration'),
                             'launch', 'robot_open_ydlidar.launch.py')
            ),
            launch_arguments={'namespace': namespace}.items()
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('namespace', default_value='limo1'),
        launch.actions.DeclareLaunchArgument('port_name', default_value='ttyTHS1'),
        launch.actions.DeclareLaunchArgument('odom_topic_name', default_value='odom'),
        launch.actions.DeclareLaunchArgument('open_rviz', default_value='false'),
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    generate_launch_description()