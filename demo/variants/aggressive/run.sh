#!/bin/bash
#
# Gap Follower - AGGRESSIVE variant
# MAX_SPEED=4.0, MIN_SPEED=2.5, SAFE_GAP=1.0
# Maximum performance with minimal safety margins
#

set -e

echo "=== Gap Follower AGGRESSIVE Variant ==="
echo "MAX_SPEED: 4.0 m/s"
echo "MIN_SPEED: 2.5 m/s"
echo "SAFE_GAP:  1.0 m"
echo ""

# Source ROS environment and pre-built gap_follower from edge container
source /opt/ros/humble/setup.bash
source /edge/install/setup.bash

# Export parameters as environment variables
export GAP_FOLLOWER_MAX_SPEED=4.0
export GAP_FOLLOWER_MIN_SPEED=2.5
export GAP_FOLLOWER_SAFE_GAP=1.0
export GAP_FOLLOWER_STEER_RATE=8.0

echo "Starting gap_follower node (aggressive)..."
ros2 run gap_follower gap_follower
