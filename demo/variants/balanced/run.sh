#!/bin/bash
#
# Gap Follower - BALANCED variant
# MAX_SPEED=1.5, MIN_SPEED=1.0, SAFE_GAP=2.0
# Balanced performance with reasonable safety margins
#

set -e

echo "=== Gap Follower BALANCED Variant ==="
echo "MAX_SPEED: 1.5 m/s"
echo "MIN_SPEED: 1.0 m/s"
echo "SAFE_GAP:  2.0 m"
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
export GAP_FOLLOWER_MAX_SPEED=1.5
export GAP_FOLLOWER_MIN_SPEED=1.0
export GAP_FOLLOWER_SAFE_GAP=2.0
export GAP_FOLLOWER_STEER_RATE=4.5

echo "Starting gap_follower node (balanced)..."
ros2 run gap_follower gap_follower
