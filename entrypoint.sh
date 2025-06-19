#!/bin/bash
. /opt/ros/humble/setup.sh
. /edge/install/setup.sh
echo $(ls && cd /edge && ls)
ros2 launch /edge/launch/muto.launch.py
