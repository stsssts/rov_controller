#!/usr/bin/env bash

export ROS_MASTER_URI=http://rov-surface:11311
source "/home/ubuntu/rov/devel/setup.bash"

exec "$@"
