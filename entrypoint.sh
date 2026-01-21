#!/bin/bash
#
# Eclipse Muto Edge Container Entrypoint
# Launches Muto stack with vehicle name from environment or hostname
#

. /opt/ros/humble/setup.sh
. /edge/muto/install/setup.sh

# Use VEHICLE_NAME from environment, fall back to hostname
VEHICLE_NAME="${VEHICLE_NAME:-$(hostname)}"

echo "=========================================="
echo "  Eclipse Muto Edge Container"
echo "=========================================="
echo "Vehicle Name: $VEHICLE_NAME"
echo "Namespace:    org.eclipse.muto.sandbox"
echo "=========================================="

ros2 launch /edge/muto/launch/muto.launch.py \
    vehicle_name:=$VEHICLE_NAME \
    vehicle_namespace:=org.eclipse.muto.sandbox \
    muto_namespace:=${VEHICLE_NAME}_muto
