#!/bin/bash
#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#

# This file is for docker container
# ROS_DOMAIN_ID is set via docker-compose environment
source /opt/ros/humble/setup.bash
source /sim_ws/install/setup.bash

# Function to maximize RViz window once it's available
maximize_rviz() {
    sleep 5  # Wait for RViz to start
    for i in {1..10}; do
        if wmctrl -l | grep -q "RViz"; then
            wmctrl -r "RViz" -b add,maximized_vert,maximized_horz
            echo "RViz window maximized"
            break
        fi
        sleep 2
    done
}

# Start the maximize function in background
maximize_rviz &

# Launch ROS simulation
ros2 launch f1tenth_gym_ros gym_bridge_launch.py num_agent:=5