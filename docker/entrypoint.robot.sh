#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd /ws

colcon build --merge-install --symlink-install
source install/setup.bash

exec ros2 launch robot_exploration test_node.launch.py 
