# Eclipse Muto OTA Demo

This demo showcases Eclipse Muto's Over-The-Air (OTA) update capabilities using a multi-agent F1TENTH racing simulation. It demonstrates:

1. **Initial Deployment** - Deploy a driving algorithm to a fleet of vehicles
2. **OTA Updates** - Seamlessly update algorithms without downtime
3. **Automatic Rollback** - Recovery from failed deployments
4. **Fleet Management** - Deploy different configurations to individual vehicles

## Prerequisites

- Docker and Docker Compose
- ROS 2 Humble (for running deploy commands)
- Python 3.10+
- Web browser (for viewing simulation)

## Quick Start

### 1. Start the Demo

```bash
cd docs/samples/muto/blueprint-demo/ros-racer
./demo/scripts/start-demo.sh
```

This will:
- Build artifact packages for all variants
- Start an HTTP server for artifacts (port 9090)
- Start the simulation environment (Docker)
- Open noVNC visualization at http://localhost:8080/vnc.html

### 2. Open the Simulation

Open your browser to: **http://localhost:8080/vnc.html**

You should see RViz with three racecars on the Spielberg track.

### 3. Deploy Your First Stack

In a new terminal:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Deploy conservative (slow) algorithm
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_conservative.json
```

Watch the cars start moving slowly around the track!

## Demo Flow

### Phase 1: Conservative Deployment (Safe Driving)

```bash
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_conservative.json
```

**What you'll see**: Cars moving slowly with wide safety margins.

**Narration point**: "We're deploying a conservative driving algorithm via OTA. Each vehicle receives the stack, downloads and builds it, then launches the driving nodes."

### Phase 2: Balanced Update (Medium Speed)

```bash
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_balanced.json
```

**What you'll see**: Cars transition to faster driving while maintaining safety.

**Narration point**: "Now we're performing an OTA update to a faster algorithm. Notice the seamless transition - no downtime, no manual intervention."

### Phase 3: Aggressive Update (Maximum Performance)

```bash
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_aggressive.json
```

**What you'll see**: Cars driving fast, taking tighter lines.

**Narration point**: "This aggressive algorithm pushes the vehicles to their limits - faster speeds, tighter turns, smaller safety margins."

### Phase 4: Rollback Demo (Faulty Update)

```bash
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_broken.json
```

**What you'll see**: Brief pause, then cars resume at previous speed.

**Narration point**: "We just deployed a faulty update that crashes on startup. Watch... Muto detects the failure and automatically rolls back to the previous working version. The fleet is self-healing!"

### Phase 5: Fleet Heterogeneity

Deploy different algorithms to different vehicles:

```bash
# Conservative to racecar1 (lead car - plays it safe)
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_conservative.json --vehicle racecar1

# Aggressive to racecar2 (chasing car - trying to overtake)
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_aggressive.json --vehicle racecar2

# Balanced to racecar3 (middle of the pack)
python3 demo/scripts/deploy-stack.py demo/stacks/gap_follower_balanced.json --vehicle racecar3
```

**What you'll see**: Cars driving at different speeds.

**Narration point**: "In real fleet deployments, you often need different configurations for different vehicles. Each edge device independently manages its own software stack."

## Algorithm Variants

| Variant | Version | MAX_SPEED | SAFE_GAP | Behavior |
|---------|---------|-----------|----------|----------|
| **conservative** | 1.0.0 | 0.5 m/s | 3.0 m | Slow, wide safety margins |
| **balanced** | 1.1.0 | 1.5 m/s | 2.0 m | Medium speed, standard safety |
| **aggressive** | 1.2.0 | 2.5 m/s | 1.2 m | Fast, tight safety margins |
| **broken** | 1.3.0 | N/A | N/A | Intentionally fails for rollback demo |

## Directory Structure

```
demo/
├── README.md                    # This file
├── NARRATION.md                 # Presenter talking points
├── stacks/
│   ├── gap_follower_conservative.json
│   ├── gap_follower_balanced.json
│   ├── gap_follower_aggressive.json
│   └── gap_follower_broken.json
├── variants/
│   ├── conservative/run.sh
│   ├── balanced/run.sh
│   ├── aggressive/run.sh
│   └── broken/run.sh
├── artifacts/                   # Generated tar.gz packages
└── scripts/
    ├── create-packages.sh       # Builds artifact packages
    ├── deploy-stack.py          # Deploys stacks via ROS
    ├── start-demo.sh            # One-click demo launcher
    └── check-status.sh          # Status checker
```

## Troubleshooting

### Cars don't move after deployment

1. Check that the artifact server is running:
   ```bash
   curl http://localhost:9090
   ```

2. Verify ROS nodes are up:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 node list | grep muto
   ```

3. Check Muto topics:
   ```bash
   ros2 topic list | grep stack
   ```

### noVNC not loading

1. Wait longer - first build can take 2-3 minutes
2. Try `docker compose restart sim`

### Deployment timeout

The artifact server must be reachable from Docker containers. The stacks use `172.17.0.1:9090` (Docker bridge IP). Verify:

```bash
docker run --rm --network host curlimages/curl curl http://172.17.0.1:9090
```

### Check demo status

```bash
./demo/scripts/check-status.sh
```

## Stopping the Demo

```bash
# Stop Docker containers
cd docs/samples/muto/blueprint-demo/ros-racer
docker compose down

# Stop artifact server (find and kill the process)
pkill -f "python3 -m http.server 9090"
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Terminal                             │
│         python3 deploy-stack.py <stack.json>                │
└─────────────────────┬───────────────────────────────────────┘
                      │ ROS 2 Topic: /muto/stack
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                Edge Containers (racecar1/2/3)                │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │ Muto Agent  │──│   Composer   │──│  Launch Plugin   │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
│         │                │                    │             │
│         │         ┌──────┴──────┐             │             │
│         │         │  Rollback   │             │             │
│         │         │   Logic     │             │             │
│         │         └─────────────┘             │             │
│         ▼                                     ▼             │
│  ┌─────────────┐                    ┌──────────────────┐   │
│  │   Gateway   │                    │   Gap Follower   │   │
│  │   (MQTT)    │                    │     Node         │   │
│  └─────────────┘                    └────────┬─────────┘   │
└──────────────────────────────────────────────┼─────────────┘
                                               │
                      /racecar{1,2,3}/drive     │
                                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    Simulation Container                      │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │  F1TENTH    │──│  Gym Bridge  │──│     RViz         │   │
│  │    Gym      │  │    Node      │  │ (via noVNC)      │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## Next Steps

- Integrate with Eclipse Symphony for cloud-based orchestration
- Add monitoring dashboards (Prometheus/Grafana)
- Extend to physical F1TENTH vehicles
- Add more algorithm variants (PID, Pure Pursuit, MPC)
