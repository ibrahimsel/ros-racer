#!/bin/bash
#
# Gap Follower - BALANCED variant
# MAX_SPEED=2.5, MIN_SPEED=1.5, SAFE_GAP=1.8
# Balanced performance with reasonable safety margins
#

set -e

echo "=== Gap Follower BALANCED Variant ==="
echo "MAX_SPEED: 2.5 m/s"
echo "MIN_SPEED: 1.5 m/s"
echo "SAFE_GAP:  1.8 m"
echo ""

# Source ROS environment and pre-built gap_follower from edge container
source /opt/ros/humble/setup.bash
source /edge/install/setup.bash

# Export parameters as environment variables
export GAP_FOLLOWER_MAX_SPEED=2.5
export GAP_FOLLOWER_MIN_SPEED=1.5
export GAP_FOLLOWER_SAFE_GAP=1.8
export GAP_FOLLOWER_STEER_RATE=6.0

echo "Starting gap_follower node (balanced)..."
ros2 run gap_follower gap_follower
