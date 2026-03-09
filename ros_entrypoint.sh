#!/bin/bash
set -e

# Setup ROS
source /opt/ros/noetic/setup.bash
# source /root/catkin_ws/devel/setup.bash

export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

exec "$@"
