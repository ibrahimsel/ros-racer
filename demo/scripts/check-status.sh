#!/bin/bash
#
# Check status of Muto deployment and ROS nodes
#

echo "=========================================="
echo "   Eclipse Muto Demo Status"
echo "=========================================="
echo ""

# Check Docker containers
echo "Docker Containers:"
echo "------------------"
docker compose -f "$(dirname "$0")/../../docker-compose.yml" ps 2>/dev/null || echo "  Docker compose not running or not found"
echo ""

# Check ROS nodes (if ROS is sourced)
if command -v ros2 &> /dev/null; then
    echo "ROS 2 Nodes:"
    echo "------------"
    ros2 node list 2>/dev/null || echo "  No ROS nodes found (is ROS sourced?)"
    echo ""

    echo "ROS 2 Topics (muto related):"
    echo "----------------------------"
    ros2 topic list 2>/dev/null | grep -E "(muto|stack|twin)" || echo "  No muto topics found"
    echo ""
else
    echo "ROS 2 not sourced. Run: source /opt/ros/humble/setup.bash"
    echo ""
fi

# Check artifact server
echo "Artifact Server:"
echo "----------------"
if curl -s http://localhost:9090 > /dev/null 2>&1; then
    echo "  Status: Running on http://localhost:9090"
    echo "  Available artifacts:"
    curl -s http://localhost:9090 2>/dev/null | grep -o 'gap_follower_[a-z]*.tar.gz' | sort -u | sed 's/^/    - /'
else
    echo "  Status: Not running"
    echo "  Start with: cd demo/artifacts && python3 -m http.server 9090"
fi
echo ""

# Check noVNC
echo "Simulation (noVNC):"
echo "-------------------"
if curl -s http://localhost:8080 > /dev/null 2>&1; then
    echo "  Status: Running on http://localhost:8080/vnc.html"
else
    echo "  Status: Not running"
fi
echo ""

# Check Muto state files (if accessible)
MUTO_STATE_DIR="${HOME}/.muto/state"
if [ -d "${MUTO_STATE_DIR}" ]; then
    echo "Muto State Files:"
    echo "-----------------"
    for state_file in "${MUTO_STATE_DIR}"/*/state.json; do
        if [ -f "${state_file}" ]; then
            stack_name=$(dirname "${state_file}" | xargs basename)
            status=$(cat "${state_file}" 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('status','unknown'))" 2>/dev/null || echo "unknown")
            version=$(cat "${state_file}" 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('current_version','unknown'))" 2>/dev/null || echo "unknown")
            echo "  ${stack_name}: status=${status}, version=${version}"
        fi
    done
else
    echo "Muto State Files:"
    echo "-----------------"
    echo "  No state directory found at ${MUTO_STATE_DIR}"
fi
echo ""
