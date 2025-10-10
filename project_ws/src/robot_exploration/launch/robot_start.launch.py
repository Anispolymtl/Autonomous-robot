import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='namespace',
                                             default_value='limo1'),
        launch.actions.DeclareLaunchArgument(name='port_name',
                                             default_value='ttyTHS1'),
        launch.actions.DeclareLaunchArgument(name='odom_topic_name',
                                             default_value='odom'),
        launch.actions.DeclareLaunchArgument(name='open_rviz',
                                             default_value='false'),
        
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            on_exit=launch.actions.Shutdown(),
            condition=launch.conditions.IfCondition(
                launch.substitutions.LaunchConfiguration('open_rviz'))
        ),

        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['0.0','0.0','0.0','0.0','0.0', '0.0', f"{launch.substitutions.LaunchConfiguration('namespace')}/base_link",f"{launch.substitutions.LaunchConfiguration('namespace')}/imu_link"]),

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

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_exploration'),
                            'launch/robot_base.launch.py')),
            launch_arguments={
                'port_name':launch.substitutions.LaunchConfiguration('port_name'),               
                'odom_frame': f"{launch.substitutions.LaunchConfiguration('namespace')}/odom",
                'base_frame': f"{launch.substitutions.LaunchConfiguration('namespace')}/base_link",
                'odom_topic_name': f"{launch.substitutions.LaunchConfiguration('namespace')}/odom"  
            }.items()
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_exploration'),
                            'launch','robot_open_ydlidar.launch.py')),
            launch_arguments={
                'namespace': launch.substitutions.LaunchConfiguration('namespace')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
