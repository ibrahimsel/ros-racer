#!/bin/bash
#
# Gap Follower - CONSERVATIVE variant
# MAX_SPEED=1.5, MIN_SPEED=0.8, SAFE_GAP=2.5
# Safe, cautious driving with wide safety margins
#

set -e

echo "=== Gap Follower CONSERVATIVE Variant ==="
echo "MAX_SPEED: 1.5 m/s"
echo "MIN_SPEED: 0.8 m/s"
echo "SAFE_GAP:  2.5 m"
echo ""

# Source ROS environment and pre-built gap_follower from edge container
source /opt/ros/humble/setup.bash
source /edge/install/setup.bash

# Export parameters as environment variables
export GAP_FOLLOWER_MAX_SPEED=1.5
export GAP_FOLLOWER_MIN_SPEED=0.8
export GAP_FOLLOWER_SAFE_GAP=2.5
export GAP_FOLLOWER_STEER_RATE=5.0

echo "Starting gap_follower node (conservative)..."
ros2 run gap_follower gap_follower
