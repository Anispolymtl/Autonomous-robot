#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd /ws
rm -rf /build /log /install 
colcon build --merge-install
source install/setup.bash
