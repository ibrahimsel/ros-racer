# AGENT.md - AI Coding Assistant Guide

This document provides context for AI coding assistants working on the ROS Racer Blueprint project.

## Project Overview

**ROS Racer Blueprint** is a multi-agent autonomous racing simulation platform showcasing Eclipse SDV software stack capabilities. It provides:

- ROS 2 communication bridge with multi-agent support (1-4 vehicles)
- F1TENTH gym environment integration for autonomous racing simulation
- Eclipse Muto integration for Over-The-Air (OTA) updates and edge management
- RViz visualization with custom Qt/C++ plugin for race control
- Multi-vehicle coordination with lap timing and collision detection

**License:** EPL-2.0 / MIT dual licensing

## Directory Structure

```
ros-racer/
├── src/
│   ├── f1tenth_gym_ros/              # Main ROS 2 gym bridge package
│   │   ├── f1tenth_gym_ros/
│   │   │   ├── gym_bridge.py         # Core simulation-ROS interface
│   │   │   └── __init__.py
│   │   ├── launch/
│   │   │   ├── gym_bridge_launch.py  # ROS 2 launch description
│   │   │   ├── ego_racecar.xacro     # URDF model definition
│   │   │   └── gym_bridge.rviz       # RViz config
│   │   ├── config/
│   │   │   ├── sim.yaml              # Gym bridge parameters
│   │   │   └── racecar.yaml          # Vehicle configuration
│   │   ├── maps/                     # Track map files (.png, .yaml, .pgm)
│   │   └── test/                     # pytest tests (flake8, copyright, pep257)
│   └── multiagent_plugin/            # RViz C++ plugin for race control
│       ├── src/
│       │   ├── multiagent_plugin.cpp
│       │   └── multiagent_plugin.hpp
│       └── CMakeLists.txt
├── demo/                             # OTA demo with Eclipse Muto
│   ├── stacks/                       # Stack definition JSON files
│   ├── variants/                     # Driving algorithm implementations
│   └── scripts/                      # Deployment and utility scripts
├── docker-compose.yml
├── Dockerfile.sim                    # Simulation container
├── Dockerfile.edge                   # Edge device container
├── run-demo.sh                       # Interactive demo script
└── muto.repos                        # VCS import for Muto dependencies
```

## Technology Stack

### ROS 2
- **Distribution:** ROS 2 Humble (Ubuntu 22.04)
- **Middleware:** CycloneDDS (rmw_cyclonedds_cpp)
- **Key packages:** rviz2, nav2_map_server, tf2_ros, joint_state_publisher, rosbridge_server

### Python
- Python 3.11+
- F1TENTH Gym (autonomous racing simulation)
- transforms3d (quaternion/euler conversions)
- rclpy (ROS 2 Python client)
- pytest (testing)

### C++
- C++17 standard
- Qt5 (GUI framework)
- RViz Plugin API (rviz_common)
- CMake 3.8+

### Infrastructure
- Docker/Podman with compose
- noVNC (web-based VNC viewer)
- HTTP artifact server for OTA packages

## Build & Run

### Docker (Recommended)

```bash
# Start all services
docker compose up -d --build

# Access simulation via browser
http://localhost:18080/vnc.html

# Run interactive demo
./run-demo.sh
```

### Native Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-path src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Docker Services

| Service | Purpose | Port |
|---------|---------|------|
| sim | F1TENTH gym + RViz | display via noVNC |
| novnc | Web VNC viewer | 18080 |
| edge/edge2/edge3 | Muto agent nodes (racecars) | ROS 2 DDS |
| artifact-server | HTTP package distribution | 9090 |

## Testing

```bash
# Run all tests
colcon test --packages-select f1tenth_gym_ros

# Run specific test
pytest src/f1tenth_gym_ros/test/test_flake8.py
```

**Test types:**
- `test_flake8.py` - PEP 8 style linting
- `test_copyright.py` - License header validation
- `test_pep257.py` - Docstring style compliance

## Coding Conventions

### Python
- **Style:** PEP 8 (enforced via flake8)
- **Docstrings:** PEP 257 format
- **Naming:** snake_case for functions/variables, PascalCase for classes
- **Headers:** EPL-2.0 or MIT license header required
- Type hints recommended

### C++
- **Standard:** C++17
- **Compiler flags:** -Wall -Wextra -Wpedantic
- **Naming:** snake_case for methods, CamelCase for classes
- **Headers:** EPL-2.0 license header required

### ROS 2 Naming
- **Topics:** `/{namespace}{agent_id}/{topic_type}` (e.g., `/racecar1/scan`)
- **Frame IDs:** `{namespace}{agent_id}/{frame}` (e.g., `racecar1/base_link`)
- **Node names:** lowercase with underscores

## Key Architecture Patterns

### Multi-Agent Topic Structure
Each agent has namespaced topics:
- `/{ns}{i}/scan` → LiDAR data (sensor_msgs/LaserScan)
- `/{ns}{i}/odom` → Position/velocity (nav_msgs/Odometry)
- `/{ns}{i}/drive` → Control commands (ackermann_msgs/AckermannDriveStamped)

**Important:** Drive messages MUST have `header.frame_id` set to `{namespace}{agent_id}/base_link` for proper agent routing.

### Simulation Loop
```
Drive Timer (100 Hz) → Physics Step → Collision Detection → Lap Tracking
            ↓
Sensor Timer (40 Hz) → Publish scans, odometry, transforms, markers
```

### Performance Parameters
- Physics loop: 100 Hz (10ms timestep)
- Sensor publish: 40 Hz
- Lap time UI: 10 Hz
- Drive timeout: 100ms (gradual deceleration on stale commands)

### Collision Detection
- Threshold: 0.21m laser range
- Triggers disqualification and timer reset
- Requires pose reset to re-enter race

## Key Configuration Files

### sim.yaml (Gym Bridge Parameters)
- `racecar_namespace`: Base namespace for topics (default: "racecar")
- `num_agent`: Number of vehicles (1-3, default: 3)
- `map_path`: Full path to map file (without extension)
- `scan_fov`: LiDAR field of view (default: 4.7 radians)
- `scan_beams`: LiDAR beam count (default: 1080)

### racecar.yaml (Vehicle Configuration)
Per-agent vehicle names and colors.

### docker-compose.yml
- `ROS_DOMAIN_ID=2`: DDS domain isolation
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`: Middleware selection

## Key Entry Points

### Core Simulation
- `src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py` - Main simulation logic
- `src/f1tenth_gym_ros/launch/gym_bridge_launch.py` - ROS 2 launch configuration
- `src/f1tenth_gym_ros/config/sim.yaml` - Runtime parameters

### UI/Visualization
- `src/multiagent_plugin/src/multiagent_plugin.cpp` - RViz panel implementation

### OTA/Demo
- `demo/scripts/deploy-stack.py` - OTA deployment logic
- `demo/variants/*/run.sh` - Driving algorithm implementations
- `demo/stacks/*.json` - Stack definitions

### Containerization
- `docker-compose.yml` - Service orchestration
- `Dockerfile.sim` - Simulation container
- `Dockerfile.edge` - Edge device container

## Common Tasks

### Adding a New Agent Parameter
1. Add parameter declaration in `gym_bridge.py` constructor
2. Add default value in `config/sim.yaml`
3. Update launch file if needed

### Adding a New RViz Control
1. Edit `multiagent_plugin.cpp` to add Qt widget
2. Create ROS subscriber/publisher for the control
3. Connect Qt signals to ROS callbacks

### Adding a New Driving Algorithm
1. Create new variant directory in `demo/variants/`
2. Create `run.sh` with the algorithm implementation
3. Create stack JSON in `demo/stacks/`
4. Generate artifacts with `demo/scripts/generate-artifacts.sh`

### Modifying Collision Behavior
Edit `GymBridge._check_collisions()` in `gym_bridge.py`. Current threshold is 0.21m.

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | DDS domain isolation | 2 |
| `RMW_IMPLEMENTATION` | Middleware selection | rmw_cyclonedds_cpp |
| `DISPLAY` | X11 display for RViz | :0 |

## Troubleshooting

### Vehicles not responding to drive commands
- Verify `header.frame_id` matches `{namespace}{agent_id}/base_link`
- Check drive timeout (100ms) hasn't expired
- Verify agent is not in collision/disqualified state

### RViz not displaying
- Check noVNC connection at http://localhost:18080/vnc.html
- Verify X11 socket mount in docker-compose.yml
- Check `DISPLAY` environment variable

### OTA deployment failing
- Verify artifact-server is running on port 9090
- Check stack JSON checksums match artifacts
- Verify edge container has network access to artifact-server

### Notes
- Don't forget that we're aiming for high performance and we're targeting multi-platform (Linux and Mac OS based systems)
