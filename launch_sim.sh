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

# Wait for X server to be available
echo "Waiting for X server at $DISPLAY..."
for i in {1..30}; do
    if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
        echo "X server ready"
        break
    fi
    echo "Waiting for X server... ($i/30)"
    sleep 1
done

# Background task to maximize rviz when it opens
(
    for i in {1..30}; do
        sleep 1
        if wmctrl -l | grep -q "RViz"; then
            wmctrl -r "RViz" -b add,maximized_vert,maximized_horz
            echo "RViz window maximized"
            break
        fi
    done
) &

# Launch ROS simulation with configurable number of agents
NUM_AGENTS="${NUM_AGENTS:-3}"
ros2 launch f1tenth_gym_ros gym_bridge_launch.py num_agents:=$NUM_AGENTS
