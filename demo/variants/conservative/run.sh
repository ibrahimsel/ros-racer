#!/bin/bash
#
# Gap Follower - CONSERVATIVE variant
# MAX_SPEED=0.5, MIN_SPEED=0.3, SAFE_GAP=3.0
# Safe, cautious driving with wide safety margins
#

set -e

echo "=== Gap Follower CONSERVATIVE Variant ==="
echo "MAX_SPEED: 0.5 m/s"
echo "MIN_SPEED: 0.3 m/s"
echo "SAFE_GAP:  3.0 m"
echo ""

# Source ROS environment
source /opt/ros/humble/setup.bash

# Build the workspace if not already built
if [ ! -d "install" ]; then
    echo "Building workspace..."
    colcon build --symlink-install
fi

# Source the workspace
source install/setup.bash

# Export parameters as environment variables (for future parameterized version)
export GAP_FOLLOWER_MAX_SPEED=0.5
export GAP_FOLLOWER_MIN_SPEED=0.3
export GAP_FOLLOWER_SAFE_GAP=3.0

echo "Starting gap_follower node (conservative)..."
ros2 run gap_follower gap_follower
