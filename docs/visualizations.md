# Visualization Topics Reference

This document describes all visualization topics available in the F1Tenth multi-agent racing simulation. All visualizations are published as ROS2 topics and can be viewed in RViz2.

## Quick Reference

| Topic | Type | Default | Description |
|-------|------|---------|-------------|
| `/racecar_labels` | `MarkerArray` | Enabled | Name labels above each racecar |
| `/speed_markers` | `MarkerArray` | Disabled | Speed text above each racecar |
| `/velocity_arrows` | `MarkerArray` | Disabled | Velocity direction arrows |
| `/safety_zones` | `MarkerArray` | Disabled | Collision proximity indicators |
| `/race_markers` | `MarkerArray` | Enabled | Start/finish line |
| `/lap_point` | `MarkerArray` | Enabled | Checkered finish line (settable) |
| `/obstacles` | `MarkerArray` | Enabled | Spawned obstacles |
| `/racecarN/trajectory` | `Path` | Disabled | Historical path per racecar |
| `/race_stats_json` | `String` | N/A | JSON race statistics |

---

## Detailed Descriptions

### 1. Racecar Labels (`/racecar_labels`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Same as sensor publish rate (20-40 Hz)

Displays the name of each racecar floating above it. Labels are color-coded to match each racecar's assigned color.

**Marker Properties:**
- Type: `TEXT_VIEW_FACING`
- Height: 0.5m above ground
- Text: "racecar1", "racecar2", etc.

---

### 2. Speed Markers (`/speed_markers`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Same as sensor publish rate (20-40 Hz)

Displays the current speed (m/s) of each racecar as floating text. Color changes based on speed:

| Speed Range | Color |
|-------------|-------|
| 0-1 m/s | Green |
| 1-2 m/s | Yellow |
| 2+ m/s | Red |

**Marker Properties:**
- Type: `TEXT_VIEW_FACING`
- Height: 0.8m above ground
- Format: `{speed:.1f} m/s`

---

### 3. Velocity Arrows (`/velocity_arrows`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Same as sensor publish rate (20-40 Hz)

Displays velocity vectors as arrows pointing in the direction of travel. Arrow length is proportional to speed.

**Marker Properties:**
- Type: `ARROW`
- Scale: Length = speed × 0.5, width = 0.1m
- Color: Same as speed markers (green → yellow → red)

---

### 4. Safety Zones (`/safety_zones`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Same as sensor publish rate (20-40 Hz)

Transparent cylinders around each racecar indicating collision proximity. Based on minimum laser scan distance.

| Min Scan Distance | Color |
|-------------------|-------|
| > 0.5m | Green (safe) |
| 0.3m - 0.5m | Yellow (warning) |
| < 0.3m | Red (danger!) |

**Marker Properties:**
- Type: `CYLINDER`
- Radius: 0.5m
- Height: 0.1m
- Alpha: 0.3 (transparent)

---

### 5. Race Markers (`/race_markers`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Published once (latched)

Static start/finish line visualization. Includes:
- White line strip at x=-1.0, spanning y=[-2.0, 2.0]
- "START / FINISH" text label

**Marker Properties:**
- Line Type: `LINE_STRIP`
- Text Type: `TEXT_VIEW_FACING`
- Position: Fixed at origin area

---

### 6. Lap Point (`/lap_point`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Published once, re-published when moved

Checkered finish line marker that can be dynamically positioned via the multiagent plugin.

**Features:**
- Classic checkered flag pattern (black/white squares)
- "FINISH" text label
- 4m wide, 0.5m deep
- Position settable via plugin dropdown → "Set Lap Point"

**To Set Custom Lap Point:**
1. In the Multiagent Plugin, select "Set Lap Point" from dropdown
2. Use RViz's "2D Pose Estimate" tool
3. Click on the map where you want the finish line
4. Lap time calculations automatically update

---

### 7. Obstacles (`/obstacles`)

**Type:** `visualization_msgs/MarkerArray`  
**Frame:** `map`  
**Rate:** Published on change

Cylindrical obstacles spawned via RViz's "Publish Point" tool.

**Features:**
- Red semi-transparent cylinders
- Radius: 0.15m (configurable in code)
- Affect laser scans (racecars see them)
- Cause collision disqualification on contact
- Clear all via "Clear Obstacles" button in plugin

**To Spawn Obstacles:**
1. In RViz toolbar, click "Publish Point" (crosshair icon)
2. Click anywhere on the map
3. Obstacle appears at that location

---

### 8. Trajectory Paths (`/racecarN/trajectory`)

**Type:** `nav_msgs/Path`  
**Frame:** `map`  
**Rate:** 5 Hz

Historical path showing the last 500 positions of each racecar. Separate topic per racecar.

**Topics:**
- `/racecar1/trajectory`
- `/racecar2/trajectory`
- `/racecar3/trajectory`
- etc.

**Display in RViz:**
1. Add → By topic → `/racecarN/trajectory` → Path
2. Adjust line width and color as desired
3. Default: disabled to reduce visual clutter

---

### 9. Race Stats JSON (`/race_stats_json`)

**Type:** `std_msgs/String`  
**Rate:** Same as sensor publish rate

JSON-formatted comprehensive race statistics for programmatic consumption.

**Schema:**
```json
{
  "timestamp": 1234567890.123,
  "num_agents": 3,
  "simulation_running": true,
  "simulation_paused": false,
  "agents": [
    {
      "id": 1,
      "name": "racecar1",
      "position": {
        "x": 1.23,
        "y": 4.56,
        "theta": 0.78
      },
      "speed": 2.5,
      "min_scan_distance": 1.2,
      "current_lap_time": 15.3,
      "best_lap_time": 14.8,
      "disqualified": false
    }
  ]
}
```

**Used By:**
- Multiagent Plugin status indicator (Racing/Idle/Paused/Stopped)
- External monitoring tools

---

## Enabling Visualizations in RViz

Most visualizations are disabled by default to optimize performance. To enable:

1. Open RViz2 panel
2. Click "Add" button
3. Select "By topic"
4. Navigate to the desired topic
5. Select the appropriate display type (MarkerArray, Path, etc.)

**Recommended for Low-End Systems:**
- Keep only `/racecar_labels` and `/race_markers` enabled
- Disable LaserScan displays
- Set Target Frame Rate to 10 FPS

**Full Visualization Setup:**
- Enable all MarkerArray topics
- Enable trajectory paths for detailed analysis
- Useful for debugging and demo presentations

---

## Color Palette

Racecars are assigned colors from this palette (cycles for >10 agents):

| Index | Color | RGB |
|-------|-------|-----|
| 0 | Orange | (1.0, 0.5, 0.0) |
| 1 | Cyan | (0.0, 1.0, 1.0) |
| 2 | Magenta | (1.0, 0.0, 1.0) |
| 3 | Lime | (0.5, 1.0, 0.0) |
| 4 | Pink | (1.0, 0.4, 0.7) |
| 5 | Teal | (0.0, 0.5, 0.5) |
| 6 | Lavender | (0.7, 0.5, 1.0) |
| 7 | Brown | (0.6, 0.3, 0.0) |
| 8 | Olive | (0.5, 0.5, 0.0) |
| 9 | Navy | (0.0, 0.0, 0.5) |
