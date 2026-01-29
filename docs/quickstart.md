
# QuickStart


![Demo](./assets/demo.png)

## Introduction


The **Eclipse SDV Blueprints** project is a collaborative initiative led by Eclipse SDV members to bring the "software defined vehicle" concepts to life. A crucial aspect of each blueprint is to ensure users can easily reproduce it on their own. This requires well-written and highly clear documentation. Users can utilize blueprints as they are, for inspiration, or as a foundation to customize and meet their specific needs.

**ROS Racer Blueprint** is a ROS-based showcase where multi-agent autonomous racers running [F1TENTH](https://f1tenth.org/) software are orchestrated and managed by an Eclipse SDV software stack.

This project provides a ROS 2 communication bridge (inspired by [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)) with multi-agent support (up to 4 vehicles) for the F1TENTH gym environment. It is designed for use with [Eclipse Muto](https://projects.eclipse.org/projects/automotive.muto), an adaptive framework and runtime platform for dynamically composable model-driven software stacks for ROS.

## Table of Contents

- [Introduction](#introduction)
- [Quick Start](#quick-start)
- [Running the Demo](#running-the-demo)
- [Installation](#installation)
  - [Docker / Podman](#docker--podman)
  - [Native on Ubuntu](#native-on-ubuntu)
- [Configuration](#configuration)
- [ROS Topics](#ros-topics)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License and Copyright](#license-and-copyright)

## Quick Start

The fastest way to get started is using Docker or Podman:

```bash
# Clone the repository
git clone https://github.com/eclipse-sdv-blueprints/ros-racer.git
cd ros-racer

# Start the simulation
docker compose up -d --build

# Open in browser
# http://localhost:18080/vnc.html
```

## Running the Demo

The project includes an interactive demo that showcases Eclipse Muto's OTA deployment capabilities:

```bash
./run-demo.sh
```

This guided demo walks you through:

| Phase | Description |
|-------|-------------|
| **Infrastructure Setup** | Starts F1TENTH simulation with Muto edge containers |
| **Direct Deployment** | Deploy algorithms via ROS topics (Muto native) |
| **OTA Updates** | Seamless zero-downtime updates between algorithm versions |
| **Automatic Rollback** | Recovery from failed deployments |
| **Fleet Management** | Deploy different algorithms to individual vehicles |

For detailed instructions, see the [Demo Documentation](../demo/README.md).

## Installation

### Supported Systems

- Ubuntu 22.04 (native or containerized)
- macOS (via Docker/Podman)
- Windows (via Docker/Podman with WSL2)

### Docker / Podman

**Prerequisites:**
- [Docker](https://docs.docker.com/install/) or [Podman](https://podman.io/getting-started/installation) with compose support
- Web browser

**Steps:**

1. Clone the repository:
   ```bash
   git clone https://github.com/eclipse-sdv-blueprints/ros-racer.git
   cd ros-racer
   ```

2. Start the services:
   ```bash
   docker compose up -d --build
   ```

   This starts:
   | Service | Description |
   |---------|-------------|
   | **novnc** | Web-based VNC viewer (port 18080) |
   | **sim** | F1TENTH gym simulation with RViz |
   | **artifact-server** | HTTP server for stack packages |
   | **edge, edge2, edge3** | Eclipse Muto edge devices (racecars) |

3. Open the simulation in your browser:
   ```
   http://localhost:18080/vnc.html
   ```

4. If RViz doesn't appear, restart the sim service:
   ```bash
   docker compose restart sim
   ```

### Native on Ubuntu

**Prerequisites:**
- ROS 2 Humble ([installation guide](https://docs.ros.org/en/humble/Installation.html))
- F1TENTH Gym:
  ```bash
  cd $HOME
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
  ```

**Steps:**

1. Clone into your colcon workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/eclipse-sdv-blueprints/ros-racer.git
   ```

2. Configure the map path in `src/f1tenth_gym_ros/config/sim.yaml`:
   ```yaml
   map_path: /home/<user>/ros2_ws/src/ros-racer/src/f1tenth_gym_ros/maps/levine
   ```
   > Note: Do not include the file extension. Set `map_img_ext` separately if using `.pgm` instead of `.png`.

3. Install dependencies and build:
   ```bash
   source /opt/ros/humble/setup.bash
   cd ~/ros2_ws
   rosdep install --from-path src --ignore-src -r -y
   colcon build --symlink-install
   ```

4. Launch the simulation:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/local_setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

> **Tip:** For remote systems, you'll need [X11 forwarding](https://unix.stackexchange.com/questions/12755/how-to-forward-x-over-ssh-to-run-graphics-applications-remotely) to view RViz.

## Configuration

The simulation configuration is at `src/f1tenth_gym_ros/config/sim.yaml`:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `map_path` | Full path to map file (without extension) | `levine` |
| `map_img_ext` | Map image extension | `.png` |
| `num_agent` | Number of racecars (1-4) | `3` |
| `racecar_namespace` | Base namespace for vehicles | `racecar` |
| `scan_topic` | LiDAR scan topic name | `scan` |
| `odom_topic` | Odometry topic name | `odom` |
| `drive_topic` | Drive command topic name | `drive` |

## ROS Topics

### Published by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/{ns}{i}/scan` | `sensor_msgs/LaserScan` | LiDAR scan for vehicle i |
| `/{ns}{i}/odom` | `nav_msgs/Odometry` | Odometry for vehicle i |
| `/map` | `nav_msgs/OccupancyGrid` | Environment map |

Where `{ns}` is the `racecar_namespace` parameter and `{i}` is the vehicle number (1-4).

### Subscribed by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/{ns}{i}/drive` | `ackermann_msgs/AckermannDriveStamped` | Drive commands for vehicle i |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Reset vehicle pose |

**Important:** When publishing drive commands, set `header.frame_id` to `{ns}{i}/base_link` (e.g., `racecar1/base_link`).

Example drive command:
```bash
ros2 topic pub racecar1/drive ackermann_msgs/msg/AckermannDriveStamped \
  "{header: {frame_id: 'racecar1/base_link'}, drive: {speed: 1.0, steering_angle: 0.5}}"
```

## Troubleshooting

<details>
<summary><strong>RViz doesn't open or crashes</strong></summary>

If using Docker and you see Qt/xcb errors:
```
sim-1 | [rviz2-1] qt.qpa.xcb: could not connect to display novnc:0.0
```

Solution: Restart the compose services:
```bash
docker compose restart
```
</details>

<details>
<summary><strong>"Invalid frame ID 'map'" warnings</strong></summary>

This is normal during startup while TF buffers populate. Wait up to a minute for transforms to stabilize.
</details>

<details>
<summary><strong>Map not visible</strong></summary>

Verify `map_path` in `sim.yaml` is correct:
- Use the full absolute path
- Do not include the file extension
- Set `map_img_ext` if using `.pgm` instead of `.png`
</details>

<details>
<summary><strong>Racecars visible but not moving</strong></summary>

1. Ensure you're publishing to the correct topics (e.g., `/racecar1/drive`)
2. Verify `header.frame_id` is set correctly (e.g., `racecar1/base_link`)
3. Check that drive message speed and steering values are non-zero
</details>

<details>
<summary><strong>Changes not reflected after rebuild</strong></summary>

For Docker:
```bash
docker compose down
docker compose up -d --build
```

For native installation:
```bash
colcon build --symlink-install
source install/local_setup.bash
```
</details>


