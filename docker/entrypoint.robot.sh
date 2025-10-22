#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/"${ROS_DISTRO}"/setup.bash
cd /ws

colcon build --symlink-install

colcon build --merge-install --symlink-install
source install/setup.bash

