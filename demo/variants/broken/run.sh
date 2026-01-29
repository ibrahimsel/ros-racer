#!/bin/bash
#
# Gap Follower - BROKEN variant (INTENTIONALLY FAILS FOR DEMO)
# This variant is designed to fail to demonstrate Muto's rollback functionality.
#

set -e

echo "=== Gap Follower BROKEN Variant ==="
echo "VERSION: 1.3.0"
echo ""
echo "WARNING: This variant will fail intentionally for rollback demonstration"
echo ""

# Source ROS environment
source /opt/ros/humble/setup.bash

# Simulate some startup work
echo "Initializing gap follower..."
sleep 2

echo "Loading configuration..."
sleep 1

# INTENTIONAL FAILURE FOR ROLLBACK DEMO
echo ""
echo "=========================================="
echo "ERROR: Simulated critical failure!"
echo "=========================================="
echo ""
echo "The BROKEN variant has encountered an unrecoverable error."
echo "Muto should now detect this failure and trigger automatic rollback"
echo "to the previous working version."
echo ""
exit 1
