set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

cd /ws

colcon build --symlink-install

source install/setup.bash

exec ros2 run robot_exploration identify_service
