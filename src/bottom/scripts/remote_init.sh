#!/usr/bin/env bash

source "/home/ubuntu/ros_ws/devel/setup.bash"
export ROS_MASTER_URI=http://Monster:11311
exec "$@"
