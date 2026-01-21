#!/bin/bash
#
# Eclipse Muto OTA Demo - One-Click Launcher
# This script starts all required services for the demo
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DIR="${SCRIPT_DIR}/.."
ROS_RACER_DIR="${DEMO_DIR}/.."

echo "=========================================="
echo "   Eclipse Muto OTA Demo Launcher"
echo "=========================================="
echo ""

# Step 1: Build packages
echo "[1/4] Building artifact packages..."
"${SCRIPT_DIR}/create-packages.sh"
echo ""

# Step 2: Start artifact server in background
echo "[2/4] Starting artifact server on port 9090..."
cd "${DEMO_DIR}/artifacts"

# Kill any existing server on port 9090
pkill -f "python3 -m http.server 9090" 2>/dev/null || true
sleep 1

python3 -m http.server 9090 &
ARTIFACT_PID=$!
echo "  Artifact server PID: ${ARTIFACT_PID}"
echo "  URL: http://localhost:9090"
cd "${SCRIPT_DIR}"
echo ""

# Step 3: Start docker compose
echo "[3/4] Starting simulation environment..."
cd "${ROS_RACER_DIR}"
docker compose up -d --build
echo ""

# Step 4: Wait for services
echo "[4/4] Waiting for services to be ready..."
echo "  This may take up to 60 seconds on first run..."

# Wait for noVNC
echo -n "  Waiting for noVNC..."
for i in {1..30}; do
    if curl -s http://localhost:8080 > /dev/null 2>&1; then
        echo " ready!"
        break
    fi
    echo -n "."
    sleep 2
done

# Additional wait for ROS nodes to initialize
echo "  Waiting for ROS nodes to initialize..."
sleep 10

echo ""
echo "=========================================="
echo "   Demo Ready!"
echo "=========================================="
echo ""
echo "Simulation:      http://localhost:8080/vnc.html"
echo "Artifact Server: http://localhost:9090"
echo ""
echo "Deploy commands:"
echo "  # Source ROS environment first:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "  # Deploy conservative (slow, safe):"
echo "  python3 ${SCRIPT_DIR}/deploy-stack.py ${DEMO_DIR}/stacks/gap_follower_conservative.json"
echo ""
echo "  # Deploy balanced (medium speed):"
echo "  python3 ${SCRIPT_DIR}/deploy-stack.py ${DEMO_DIR}/stacks/gap_follower_balanced.json"
echo ""
echo "  # Deploy aggressive (fast):"
echo "  python3 ${SCRIPT_DIR}/deploy-stack.py ${DEMO_DIR}/stacks/gap_follower_aggressive.json"
echo ""
echo "  # Deploy broken (triggers rollback):"
echo "  python3 ${SCRIPT_DIR}/deploy-stack.py ${DEMO_DIR}/stacks/gap_follower_broken.json"
echo ""
echo "  # Deploy to specific vehicle:"
echo "  python3 ${SCRIPT_DIR}/deploy-stack.py ${DEMO_DIR}/stacks/gap_follower_conservative.json --vehicle racecar1"
echo ""
echo "To stop the demo:"
echo "  cd ${ROS_RACER_DIR} && docker compose down"
echo "  kill ${ARTIFACT_PID}  # Stop artifact server"
echo ""
