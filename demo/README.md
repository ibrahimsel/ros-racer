# Eclipse Muto OTA Demo

This demo showcases Eclipse Muto's Over-The-Air (OTA) update capabilities using a multi-agent F1TENTH racing simulation.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Interactive Demo Script](#interactive-demo-script)
- [Manual Deployment](#manual-deployment)
- [Algorithm Variants](#algorithm-variants)
- [Architecture](#architecture)
- [Directory Structure](#directory-structure)
- [Troubleshooting](#troubleshooting)

## Features

- **Initial Deployment** - Deploy a driving algorithm to a fleet of vehicles
- **OTA Updates** - Seamlessly update algorithms without downtime
- **Automatic Rollback** - Recovery from failed deployments
- **Fleet Management** - Deploy different configurations to individual vehicles

## Prerequisites

- **Docker** or **Podman** (with compose support)
- **curl**, **jq**, **base64** (for the demo script)
- Web browser (for viewing simulation via noVNC)

> **Note**: ROS 2 is NOT required on your host machine. All ROS operations run inside the containers.

## Quick Start

### Option 1: Interactive Demo (Recommended)

From the repository root:

```bash
./run-demo.sh
```

This interactive script walks you through the complete demo step-by-step, explaining each phase.

### Option 2: Manual Setup

1. **Start the containers**:
   ```bash
   docker compose up -d --build
   ```

2. **Open the simulation** in your browser:
   ```
   http://localhost:18080/vnc.html
   ```

3. **Deploy an algorithm** (from inside an edge container):
   ```bash
   docker exec ros-racer-edge-1 bash -c "source /opt/ros/humble/setup.bash && source /edge/muto/install/setup.bash && python3 /edge/demo/scripts/deploy-stack.py /edge/demo/stacks/gap_follower_conservative.json"
   ```

## Interactive Demo Script

The `run-demo.sh` script at the repository root provides a guided walkthrough:

```bash
./run-demo.sh
```

### What the Demo Covers

| Phase | Description |
|-------|-------------|
| **Prerequisites** | Checks for Docker/Podman, curl, jq, base64 |
| **Infrastructure** | Starts simulation and edge containers |
| **Conservative Deploy** | Deploys slow, safe driving algorithm |
| **Balanced Update** | OTA update to medium-speed algorithm |
| **Aggressive Update** | OTA update to high-performance algorithm |
| **Rollback Demo** | Deploys faulty update, demonstrates auto-recovery |
| **Fleet Heterogeneity** | Different algorithms per vehicle |

## Manual Deployment

To deploy stacks manually, execute commands inside an edge container:

```bash
# Deploy conservative (slow) algorithm to all vehicles
docker exec ros-racer-edge-1 bash -c "\
  source /opt/ros/humble/setup.bash && \
  source /edge/muto/install/setup.bash && \
  python3 /edge/demo/scripts/deploy-stack.py /edge/demo/stacks/gap_follower_conservative.json"

# Deploy to a specific vehicle only
docker exec ros-racer-edge-1 bash -c "\
  source /opt/ros/humble/setup.bash && \
  source /edge/muto/install/setup.bash && \
  python3 /edge/demo/scripts/deploy-stack.py /edge/demo/stacks/gap_follower_aggressive.json --vehicle racecar2"
```

## Algorithm Variants

| Variant | Version | Speed | Safety Gap | Behavior |
|---------|---------|-------|------------|----------|
| **conservative** | 1.0.0 | 0.5 m/s | 3.0 m | Slow, wide safety margins |
| **balanced** | 1.1.0 | 1.5 m/s | 2.0 m | Medium speed, standard safety |
| **aggressive** | 1.2.0 | 2.5 m/s | 1.2 m | Fast, tight safety margins |
| **broken** | 1.3.0 | N/A | N/A | Intentionally crashes for rollback demo |

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Host Machine                                 │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              Docker/Podman Compose                        │   │
│  │                                                           │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │   │
│  │  │    edge     │  │    edge2    │  │    edge3    │       │   │
│  │  │ (racecar1)  │  │ (racecar2)  │  │ (racecar3)  │       │   │
│  │  │             │  │             │  │             │       │   │
│  │  │ Muto Agent  │  │ Muto Agent  │  │ Muto Agent  │       │   │
│  │  │ + Composer  │  │ + Composer  │  │ + Composer  │       │   │
│  │  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘       │   │
│  │         │                │                │              │   │
│  │         └────────────────┼────────────────┘              │   │
│  │                          │ ROS 2 DDS                     │   │
│  │                          ▼                               │   │
│  │  ┌───────────────────────────────────────────────────┐   │   │
│  │  │                    sim                             │   │   │
│  │  │  F1TENTH Gym + Gym Bridge + RViz                  │   │   │
│  │  └───────────────────────────────────────────────────┘   │   │
│  │                          │                               │   │
│  │                          ▼                               │   │
│  │  ┌───────────────────────────────────────────────────┐   │   │
│  │  │                   novnc                            │   │   │
│  │  │  Web-based VNC viewer (port 18080)                │   │   │
│  │  └───────────────────────────────────────────────────┘   │   │
│  │                                                           │   │
│  │  ┌───────────────────────────────────────────────────┐   │   │
│  │  │              artifact-server                       │   │   │
│  │  │  HTTP server for stack packages (port 9090)       │   │   │
│  │  └───────────────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  Browser: http://localhost:18080/vnc.html                       │
└─────────────────────────────────────────────────────────────────┘
```

### Deployment Flow

1. Stack definition is published to ROS topic `/muto/stack`
2. Each Muto agent receives the message
3. Composer downloads the algorithm package from the artifact server
4. Workspace is built and the gap_follower node is launched
5. On failure, automatic rollback to previous working version

## Directory Structure

```
demo/
├── README.md              # This file
├── stacks/                # Stack definition JSON files
│   ├── gap_follower_conservative.json
│   ├── gap_follower_balanced.json
│   ├── gap_follower_aggressive.json
│   └── gap_follower_broken.json
├── variants/              # Algorithm implementations
│   ├── conservative/run.sh
│   ├── balanced/run.sh
│   ├── aggressive/run.sh
│   └── broken/run.sh
├── artifacts/             # Generated tar.gz packages (auto-created)
└── scripts/
    ├── deploy-stack.py    # Deploys stacks via ROS topics
    ├── create-packages.sh # Builds artifact packages
    ├── generate-artifacts.sh
    └── check-status.sh    # Status checker
```

## Troubleshooting

### Cars don't move after deployment

1. Check container logs:
   ```bash
   docker compose logs edge
   ```

2. Verify the artifact server is running:
   ```bash
   curl http://localhost:9090
   ```

3. Check Muto state inside a container:
   ```bash
   docker exec ros-racer-edge-1 bash -c "ls ~/.muto/workspaces"
   ```

### noVNC not loading

1. Wait for containers to fully start (first build can take 2-3 minutes)
2. Try restarting the sim service:
   ```bash
   docker compose restart sim
   ```

### Deployment timeout

Ensure the artifact server container is healthy:

```bash
docker compose ps artifact-server
```

### Check overall status

```bash
./demo/scripts/check-status.sh
```

## Stopping the Demo

```bash
# From the repository root
docker compose down
```

Or if using the interactive demo, select "Yes" when prompted at the end.
