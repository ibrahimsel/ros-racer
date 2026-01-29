# Implementation Plan: ROS Racer Improvements

Based on [SUGGESTIONS.md](SUGGESTIONS.md) - targeting **high performance** and **multi-platform** (Linux & macOS).

---

## Overview

Three major improvement areas:
1. **F1tenth_gym submodule** + hitbox/laserscan accuracy research
2. **Multiagent plugin UI** improvements and new features
3. **Bridge performance** optimization (scale to 20 agents) + visualizations

---

## 1. F1tenth_gym Submodule Integration

### 1.1 Add Submodule

```bash
git submodule add https://github.com/ibrahimsel/f1tenth_gym.git external/f1tenth_gym
```

### 1.2 Modify Dockerfile.sim

**File:** [Dockerfile.sim](Dockerfile.sim) (lines 56-59)

Replace git clone with:
```dockerfile
# Install uv for fast package management
RUN pip3 install uv

COPY ./external/f1tenth_gym /f1tenth_gym
RUN cd /f1tenth_gym && uv pip install --system -e .
```

**Note:** `uv` is significantly faster than pip and provides better dependency resolution.

### 1.3 Multi-Platform Consideration

- Ensure submodule instructions work on both Linux and macOS
- Document `git clone --recurse-submodules` workflow
- Pin to specific commit for reproducibility

---

## 2. Automatic Agent Spawn Positioning

### 2.1 Current Problem

**File:** [gym_bridge.py](src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py) (lines 153-158)

Current spawn logic is hardcoded:
```python
self.pose_reset_arr = np.zeros((self.num_agents, 3))
for i in range(len(self.pose_reset_arr)):
    self.pose_reset_arr[i][0] += i  # x = 0, 1, 2, ... (1m apart)
```

**Issues:**
- Agents always spawn at (0,0), (1,0), (2,0), etc.
- No validation against map's occupancy grid
- Users must manually adjust when maps change
- May spawn inside walls on certain maps

### 2.2 Solution: Map-Aware Spawn System

#### Step 1: Map Analysis Module

Create `spawn_utils.py` in `src/f1tenth_gym_ros/f1tenth_gym_ros/`:

```python
import numpy as np
import yaml
from PIL import Image

class MapSpawnFinder:
    def __init__(self, map_yaml_path: str):
        # Load map metadata
        with open(map_yaml_path, 'r') as f:
            self.meta = yaml.safe_load(f)

        self.resolution = self.meta['resolution']  # meters per pixel
        self.origin = self.meta['origin']          # [x, y, theta]
        self.free_thresh = self.meta.get('free_thresh', 0.196)
        self.occupied_thresh = self.meta.get('occupied_thresh', 0.65)

        # Load occupancy grid image
        map_dir = os.path.dirname(map_yaml_path)
        img_path = os.path.join(map_dir, self.meta['image'])
        self.grid = np.array(Image.open(img_path).convert('L')) / 255.0

        # Optionally negate
        if self.meta.get('negate', 0):
            self.grid = 1.0 - self.grid

    def world_to_pixel(self, x: float, y: float) -> tuple:
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        # Flip Y axis (image origin is top-left)
        py = self.grid.shape[0] - 1 - py
        return (px, py)

    def pixel_to_world(self, px: int, py: int) -> tuple:
        # Flip Y axis
        py = self.grid.shape[0] - 1 - py
        x = px * self.resolution + self.origin[0]
        y = py * self.resolution + self.origin[1]
        return (x, y)

    def is_free(self, x: float, y: float, radius: float = 0.3) -> bool:
        """Check if position is free with safety margin."""
        px, py = self.world_to_pixel(x, y)
        r_px = int(radius / self.resolution)

        # Check bounding box
        for dx in range(-r_px, r_px + 1):
            for dy in range(-r_px, r_px + 1):
                if dx*dx + dy*dy <= r_px*r_px:
                    cx, cy = px + dx, py + dy
                    if not (0 <= cx < self.grid.shape[1] and
                            0 <= cy < self.grid.shape[0]):
                        return False
                    if self.grid[cy, cx] < self.free_thresh:
                        return False
        return True

    def find_spawn_positions(self, num_agents: int,
                             min_distance: float = 1.5,
                             vehicle_radius: float = 0.3) -> list:
        """Find valid spawn positions for N agents."""
        # Find all free pixels
        free_mask = self.grid >= self.free_thresh
        free_coords = np.argwhere(free_mask)

        if len(free_coords) == 0:
            raise ValueError("No free space found in map")

        # Find centroid of free space (likely track center)
        centroid_px = free_coords.mean(axis=0).astype(int)
        centroid_world = self.pixel_to_world(centroid_px[1], centroid_px[0])

        # Strategy: spread agents around centroid
        spawn_positions = []
        angle_step = 2 * np.pi / num_agents
        search_radius = 2.0  # Start searching 2m from centroid

        for i in range(num_agents):
            angle = i * angle_step
            found = False

            for r in np.arange(0, 20, 0.5):  # Search outward
                x = centroid_world[0] + r * np.cos(angle)
                y = centroid_world[1] + r * np.sin(angle)

                if self.is_free(x, y, vehicle_radius):
                    # Check distance from other spawns
                    too_close = False
                    for sx, sy, _ in spawn_positions:
                        if np.sqrt((x-sx)**2 + (y-sy)**2) < min_distance:
                            too_close = True
                            break

                    if not too_close:
                        # Orient toward centroid (race start direction)
                        theta = np.arctan2(centroid_world[1] - y,
                                          centroid_world[0] - x)
                        spawn_positions.append((x, y, theta))
                        found = True
                        break

            if not found:
                # Fallback: just find any free spot
                for _ in range(1000):
                    idx = np.random.randint(len(free_coords))
                    py, px = free_coords[idx]
                    x, y = self.pixel_to_world(px, py)
                    if self.is_free(x, y, vehicle_radius):
                        spawn_positions.append((x, y, 0.0))
                        break

        return spawn_positions
```

#### Step 2: Integrate into gym_bridge.py

Replace hardcoded spawn (lines 153-158) with:

```python
from f1tenth_gym_ros.spawn_utils import MapSpawnFinder

# In __init__, after map_path is resolved:
try:
    spawn_finder = MapSpawnFinder(self.map_path + ".yaml")
    spawn_positions = spawn_finder.find_spawn_positions(
        self.num_agents,
        min_distance=1.5,
        vehicle_radius=0.22  # Match collision_models.py width/2
    )

    self.pose_reset_arr = np.array(spawn_positions)
    self.get_logger().info(f"Auto-spawned {self.num_agents} agents at valid positions")

except Exception as e:
    self.get_logger().warn(f"Auto-spawn failed: {e}, using fallback")
    # Fallback to original hardcoded logic
    self.pose_reset_arr = np.zeros((self.num_agents, 3))
    for i in range(self.num_agents):
        self.pose_reset_arr[i][0] = i
```

#### Step 3: Add Configuration Parameter

**File:** [sim.yaml](src/f1tenth_gym_ros/config/sim.yaml)

```yaml
gym_bridge:
  ros__parameters:
    # Spawn configuration
    spawn_strategy: "auto"      # "auto", "manual", "grid", "line"
    spawn_min_distance: 1.5     # Minimum distance between agents (meters)
    spawn_vehicle_radius: 0.22  # Safety radius for collision checking
```

### 2.3 Spawn Strategies

| Strategy | Description |
|----------|-------------|
| `auto` | Find centroid of free space, distribute agents radially |
| `manual` | Use hardcoded positions (current behavior) |
| `grid` | Place agents in a grid pattern in largest free region |
| `line` | Place agents in a line at starting position |

### 2.4 Map-Specific Spawn Hints (Optional)

Allow maps to define preferred spawn areas in YAML:

```yaml
# In levine.yaml (extended format)
image: levine.png
resolution: 0.050000
origin: [-51.224998, -51.224998, 0.000000]
# ... existing fields ...

# Optional spawn hints
spawn_region:
  x_min: -10.0
  x_max: 10.0
  y_min: -5.0
  y_max: 5.0
  default_theta: 1.57  # Face direction (radians)
```

---

## 3. Hitbox & LaserScan Research (Continuation)

### 2.1 Dimensional Discrepancy Found

| Source | Length | Width |
|--------|--------|-------|
| gym_bridge.py collision threshold | 0.21m (isotropic) | 0.21m |
| f1tenth_gym collision_models.py | 0.32m | 0.22m |
| ego_racecar.xacro visualization | 0.3302m | 0.2032m |

### 2.2 Proposed Fix - Angle-Dependent Collision

**File:** [gym_bridge.py](src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py) (line 232)

Current simple threshold is inaccurate. Implement geometry-aware collision:
```python
def handle_collision(self):
    half_length, half_width = 0.16, 0.11  # Match GJK hitbox
    for i in range(self.num_agents):
        for beam_idx, dist in enumerate(self.obs["scans"][i]):
            angle = self.angle_min + beam_idx * self.angle_inc
            # Calculate distance to vehicle boundary at this angle
            min_dist = min(half_length / max(abs(np.cos(angle)), 1e-6),
                          half_width / max(abs(np.sin(angle)), 1e-6))
            if dist <= min_dist:
                self.agent_disqualified[i] = True
                break
```

### 2.3 LaserScan Fixes

**File:** [sim.yaml](src/f1tenth_gym_ros/config/sim.yaml)

```yaml
scan_fov: 4.712389          # Real Hokuyo UST-10LX (270°)
scan_distance_to_base_link: 0.275  # Match xacro laser position
```

---

## 3. Multiagent Plugin UI Improvements

### 3.1 Fix Existing Bugs

**File:** [multiagent_plugin.cpp](src/multiagent_plugin/src/multiagent_plugin.cpp)

- `reset_publisher_` declared but never initialized → implement or remove
- Dropdown hardcoded to 3 agents → make dynamic

### 3.2 New UI Layout with Styling

Modernize with Qt5 GroupBoxes and styling:
- **Status Section**: LED indicator (green/yellow/red) + state label
- **Race Control**: Styled Start/Pause/Reset buttons
- **Agent Selection**: Dynamic dropdown based on `num_agents`
- **Lap Times**: Monospace terminal-style display

### 3.3 New Features

1. **Reset Functionality**
   - Add `sim_reset` topic to [gym_bridge.py](src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py)
   - Reset all agents to start positions, clear disqualifications

2. **Simulation State Feedback**
   - Subscribe to sim state, update UI indicator in real-time

3. **Fix Stale Start Button**
   - Robot descriptions published at launch but RViz may miss them
   - Ensure `robot_state_publisher` uses `transient_local` QoS durability

### 3.4 Multi-Platform (C++ Plugin)

- Use Qt5 (already cross-platform)
- Avoid platform-specific APIs
- Test on both Linux and macOS RViz builds

---

## 4. Bridge Performance Optimization

**Target:** Scale to 20 agents without performance degradation

### 4.1 Optimized Collision Detection

**File:** [gym_bridge.py](src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py) (lines 230-239)

Current: O(n × 1080) comparisons per tick

Optimized with NumPy vectorization:
```python
def handle_collision(self):
    min_scans = np.min(self.obs["scans"], axis=1)  # Vectorized!
    for i in range(self.num_agents):
        if not self.agent_disqualified[i] and min_scans[i] <= threshold:
            self.agent_disqualified[i] = True
```

**Performance gain:** 21,600 comparisons → 1 vectorized operation for 20 agents

### 4.2 Batch TF Transform Broadcasting

Current: 4 separate `sendTransform()` calls per agent

Optimized: Collect all transforms, single batch call
```python
transforms = []
for i in range(self.num_agents):
    transforms.extend([base_tf, laser_tf, wheel_left_tf, wheel_right_tf])
self.br.sendTransform(transforms)  # Single call
```

### 4.3 Pre-allocated Message Objects

Avoid allocating `LaserScan`, `Odometry`, `Marker` objects every callback:
```python
# In __init__:
self._scan_msgs = [LaserScan() for _ in range(self.num_agents)]
self._odom_msgs = [Odometry() for _ in range(self.num_agents)]
```

### 4.4 Adaptive Publishing Rates

```python
if self.num_agents <= 5:
    sensor_rate = 40  # Hz
elif self.num_agents <= 10:
    sensor_rate = 25  # Hz
else:
    sensor_rate = 20  # Hz
```

### 4.5 Dynamic Agent Colors

**File:** [racecar.yaml](src/f1tenth_gym_ros/config/racecar.yaml)

Add color palette for 20 agents:
```yaml
default_colors: ['black', 'green', 'red', 'blue', 'yellow', 'purple', 'cyan', 'orange', 'pink', 'brown']
```

---

## 5. New Visualizations

### 5.1 Trajectory History (Path)

- Track last 500 positions per agent
- Publish as `nav_msgs/Path` at 5 Hz
- Color-coded per agent

### 5.2 Velocity Vectors

- Arrow markers showing speed and direction
- Color gradient: green (slow) → red (fast)

### 5.3 Safety Zones

- Transparent circles around each agent
- Changes color when approaching collision threshold

### 5.4 Lap/Checkpoint Markers

- Start/finish line visualization
- Text marker for "START/FINISH"

### 5.5 Enhanced Race Stats (JSON)

Publish comprehensive stats for UI consumption:
```json
{
  "agents": [{
    "id": 1,
    "speed": 5.2,
    "min_scan_distance": 1.3,
    "current_lap_time": 12.5,
    "best_lap_time": 11.2,
    "disqualified": false
  }]
}
```

---

## Files to Modify

| File | Changes |
|------|---------|
| `Dockerfile.sim` | Use submodule instead of git clone, add uv |
| `gym_bridge.py` | Auto-spawn integration, collision optimization, batch TF, pre-allocation, visualizations, reset handler |
| `sim.yaml` | Fix scan_fov, scan_distance_to_base_link, add spawn & visualization params |
| `racecar.yaml` | Dynamic color palette for 20 agents |
| `multiagent_plugin.cpp` | Complete UI rewrite with groups, styling, state feedback |
| `multiagent_plugin.hpp` | New member variables for UI elements and publishers |
| `gym_bridge.rviz` | Add displays for trajectories, velocity, safety zones |
| Map YAML files | Optional: Add spawn_region hints |

## Files to Create

| File | Purpose |
|------|---------|
| `.gitmodules` | Auto-created by submodule add |
| `external/f1tenth_gym/` | Submodule directory |
| `spawn_utils.py` | Map analysis and automatic spawn positioning |

---

## Implementation Checklist

### Phase 1: Submodule Setup
- [x] Add f1tenth_gym submodule (`git submodule add https://github.com/ibrahimsel/f1tenth_gym.git external/f1tenth_gym`)
- [x] Update Dockerfile.sim to use submodule + uv
- [ ] Test Docker build on Linux
- [ ] Test Docker build on macOS

### Phase 2: Automatic Spawn Positioning
- [x] Create `spawn_utils.py` in `src/f1tenth_gym_ros/f1tenth_gym_ros/`
- [x] Implement `MapSpawnFinder.__init__()` - load YAML metadata
- [x] Implement `MapSpawnFinder.world_to_pixel()` and `pixel_to_world()`
- [x] Implement `MapSpawnFinder.is_free()` - collision checking with radius
- [x] Implement `MapSpawnFinder.find_spawn_positions()` - distribute agents
- [x] Integrate into `gym_bridge.py` `__init__` (replace hardcoded spawn)
- [x] Add spawn configuration parameters to `sim.yaml`
- [ ] Test on levine map
- [ ] Test on example_map
- [ ] Test on Spielberg_map
- [ ] Test on levine_blocked map

### Phase 3: Performance Optimizations
- [x] Implement vectorized collision detection (`np.min()` instead of loop)
- [x] Implement batch TF transform broadcasting (single `sendTransform()` call)
- [ ] Pre-allocate `LaserScan` message objects
- [ ] Pre-allocate `Odometry` message objects
- [ ] Pre-allocate `Marker` objects for labels
- [x] Implement adaptive publishing rates based on agent count
- [x] Update `racecar.yaml` with dynamic color palette for 20 agents
- [ ] Benchmark with 5 agents
- [ ] Benchmark with 10 agents
- [ ] Benchmark with 20 agents

### Phase 4: Hitbox/LaserScan Research
- [ ] Examine f1tenth_gym `collision_models.py` in submodule
- [ ] Document actual GJK hitbox dimensions
- [ ] Implement angle-dependent collision threshold (or decide to keep simple)
- [x] Fix `scan_fov` in sim.yaml (4.7 → 4.712389)
- [x] Fix `scan_distance_to_base_link` in sim.yaml (0.0 → 0.275)
- [ ] Test laser accuracy with known obstacles

### Phase 5: Multiagent Plugin UI
- [x] Fix `reset_publisher_` bug (implement or remove)
- [x] Fix hardcoded 3 agents in dropdown → dynamic (now supports 10)
- [x] Add `sim_reset` topic to gym_bridge.py
- [x] Add `reset_callback()` to gym_bridge.py
- [x] Create new UI layout with QGroupBox sections
- [x] Add status indicator (LED-style green/yellow/red)
- [x] Add simulation state label
- [x] Style Start button (green)
- [x] Style Pause button (blue)
- [x] Add Reset button (orange)
- [x] Style lap times display (monospace terminal-style)
- [x] Add simulation state subscription and feedback
- [ ] Test on Linux RViz
- [ ] Test on macOS RViz

### Phase 6: Visualizations
- [x] Create `TrajectoryTracker` class
- [x] Add trajectory publishers per agent
- [x] Implement trajectory Path publishing at 5 Hz
- [x] Add velocity vector MarkerArray publisher
- [x] Implement velocity arrows (color gradient by speed)
- [x] Add safety zone MarkerArray publisher
- [x] Implement safety zone circles (color by proximity)
- [ ] Add lap marker publisher
- [ ] Implement start/finish line visualization
- [ ] Add race stats JSON publisher
- [ ] Update `gym_bridge.rviz` with new displays
- [ ] Test all visualizations together

---

## Verification

### Performance Testing
```bash
# Build and launch with 20 agents
colcon build --packages-select f1tenth_gym_ros multiagent_plugin
ros2 launch f1tenth_gym_ros gym_bridge_launch.py num_agent:=20

# Monitor CPU/memory usage
htop  # or Activity Monitor on macOS
ros2 topic hz /racecar1/scan  # Verify publish rates
```

### Multi-Platform Testing
- Build on Ubuntu 22.04 (Linux)
- Build on macOS with ROS 2 Humble
- Verify RViz plugin loads on both platforms
- Test Docker compose on both (Docker Desktop for macOS)

### Functional Testing
1. Start simulation → verify all 20 agents spawn **in valid free space**
2. Test auto-spawn on different maps (levine, example_map, Spielberg)
3. Verify agents don't spawn inside walls or obstacles
4. Test collision detection accuracy
5. Verify reset functionality
6. Check trajectory visualization
7. Confirm lap timing works for all agents

### Auto-Spawn Testing
```bash
# Test with different maps
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:=levine num_agent:=5
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:=Spielberg_map num_agent:=10
ros2 launch f1tenth_gym_ros gym_bridge_launch.py map:=example_map num_agent:=20

# Verify spawn positions in logs
# Should see: "Auto-spawned N agents at valid positions"
```

---

## Multi-Platform Considerations

| Component | Linux | macOS |
|-----------|-------|-------|
| Docker | Native | Docker Desktop |
| ROS 2 | Native packages | Homebrew/source |
| Qt5 | System packages | Homebrew Qt |
| Python packages | uv | uv |
| f1tenth_gym | uv pip install | uv pip install |
| X11/Display | Native | XQuartz |

### Key Notes:
- **Use `uv` for all Python package management** - faster and cross-platform
- Use CMake portable constructs (no Linux-specific flags)
- Avoid hardcoded paths (use `ament_index` for package paths)
- Test on both CycloneDDS and FastDDS
- Docker containers target linux/amd64 AND linux/arm64 (M1/M2 Macs)

### uv Integration
Replace pip commands throughout with uv equivalents:
```bash
# Instead of: pip install -r requirements.txt
uv pip install --system -r requirements.txt

# Instead of: pip install -e .
uv pip install --system -e .
```

Benefits:
- 10-100x faster than pip
- Better dependency resolution
- Cross-platform (Linux, macOS, Windows)
- Drop-in pip replacement
