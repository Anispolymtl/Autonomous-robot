#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd /ws

colcon build --merge-install --symlink-install
source install/setup.bash

