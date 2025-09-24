# birdro_planner

A ROS 2 free space based path planner designed for autonomous parking and low-speed maneuvering scenarios. This package provides intelligent trajectory planning with obstacle avoidance, speed constraints, and no-driving zone support.

## Overview

The `birdro_planner` package consists of two main components:

1. **BirdroPlanner**: The main planning node that generates collision-free trajectories using configurable algorithms (A* or RRT*)
2. **NDZCostmapOverlay**: A costmap overlay node that applies no-driving zones from OpenStreetMap data

### Key Features

- **Multiple Planning Algorithms**: Support for both A* and RRT* path planning algorithms
- **Adaptive Speed Control**: Built-in speed guard with acceleration and velocity limits
- **Velocity Ramping**: Smooth acceleration/deceleration profiles with reverse gate protection
- **No-Driving Zone Support**: Integration with OSM data to define restricted areas
- **Real-time Replanning**: Dynamic obstacle detection and path replanning
- **Safety Guards**: Multiple safety mechanisms including speed limits and course-out detection

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Goal Pose     │    │  Occupancy Grid  │    │   Vector Map    │
│                 │    │                  │    │                 │
└─────────┬───────┘    └─────────┬────────┘    └─────────┬───────┘
          │                      │                       │
          ▼                      ▼                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                    BirdroPlanner                                │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────────┐ │
│  │   A* / RRT* │  │ Speed Guard  │  │   Velocity Ramping      │ │
│  │   Planning  │  │   System     │  │      System             │ │
│  └─────────────┘  └──────────────┘  └─────────────────────────┘ │
└─────────────────────┬───────────────────────────────────────────┘
                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Trajectory                                   │
└─────────────────────────────────────────────────────────────────┘
```

## Installation

### Dependencies

This package requires ROS 2 and depends on several Autoware packages:

- `rclcpp` and `rclcpp_components`
- `autoware_freespace_planning_algorithms`
- `autoware_planning_msgs`
- `autoware_motion_utils`
- `autoware_universe_utils`
- `autoware_map_msgs`
- `autoware_lanelet2_extension`
- `tier4_map_msgs`
- `geography_utils`
- `lanelet2_*` packages
- `tinyxml2_vendor`

### Build Instructions

```bash
# Navigate to your ROS 2 workspace
cd ~/your_ros2_ws

# Clone the repository
git clone https://github.com/jwoak00/birdro_planner.git src/birdro_planner

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select birdro_planner

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Planner

Use the provided launch file to start both the planner and NDZ overlay:

```bash
ros2 launch birdro_planner birdro_planner.launch.xml
```

### Launch Parameters

Key parameters that can be configured:

```bash
# Planning algorithm selection
ros2 launch birdro_planner birdro_planner.launch.xml algorithm_name:=astar  # or rrtstar

# Speed limits
ros2 launch birdro_planner birdro_planner.launch.xml speed_guard.max_speed_mps:=2.0

# Enable/disable no-driving zones
ros2 launch birdro_planner birdro_planner.launch.xml ndz.enable:=true
```

## Nodes

### birdro_planner_node

The main planning node that generates trajectories.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/input/occupancy_grid` | `nav_msgs/OccupancyGrid` | Costmap for obstacle detection |
| `~/input/odometry` | `nav_msgs/Odometry` | Vehicle state information |
| `~/input/vector_map` | `autoware_map_msgs/LaneletMapBin` | HD map data |
| `/planning/mission_planning/goal` | `geometry_msgs/PoseStamped` | Target goal pose |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/output/trajectory` | `autoware_planning_msgs/Trajectory` | Generated trajectory |
| `~/debug/pose_array` | `geometry_msgs/PoseArray` | Debug visualization |
| `~/debug/goal_marker` | `visualization_msgs/Marker` | Goal visualization |

#### Parameters

**Core Planning Parameters:**
- `algorithm_name` (string, default: "astar"): Planning algorithm ("astar" or "rrtstar")
- `cruise_velocity` (double, default: 1.39): Target cruise velocity [m/s]
- `update_rate` (double, default: 10.0): Planning update frequency [Hz]
- `th_arrived_distance_m` (double, default: 1.0): Goal arrival threshold [m]

**Speed Guard Parameters:**
- `speed_guard.enable` (bool, default: true): Enable speed guard system
- `speed_guard.max_speed_mps` (double, default: 1.5): Maximum allowed speed [m/s]
- `speed_guard.max_accel_mps2` (double, default: 2.5): Maximum acceleration [m/s²]
- `speed_guard.max_lateral_accel_mps2` (double, default: 4.0): Maximum lateral acceleration [m/s²]

**Velocity Ramping Parameters:**
- `ramp.enable` (bool, default: true): Enable velocity ramping
- `ramp.max_accel_mps2` (double, default: 1.0): Ramping acceleration limit [m/s²]
- `ramp.max_decel_mps2` (double, default: 1.0): Ramping deceleration limit [m/s²]
- `ramp.reverse_gate.enable` (bool, default: true): Enable reverse direction protection

### ndz_costmap_overlay

Overlays no-driving zones onto the costmap using OSM data.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `input_topic` | `nav_msgs/OccupancyGrid` | Input costmap |
| `map_topic` | `nav_msgs/OccupancyGrid` | Reference map |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `output_topic` | `nav_msgs/OccupancyGrid` | Costmap with NDZ overlay |

#### Parameters

**NDZ Configuration:**
- `ndz.enable` (bool, default: true): Enable no-driving zone overlay
- `ndz.osm_path` (string): Path to OSM file containing NDZ data
- `ndz.way_id` (int64, default: -10000): OSM way ID for no-driving zones
- `ndz.ref_lat` (double): Reference latitude for coordinate conversion
- `ndz.ref_lon` (double): Reference longitude for coordinate conversion

## Algorithm Selection

### A* (Default)
- **Pros**: Fast, deterministic, optimal for grid-based planning
- **Cons**: Limited to discrete search space
- **Best for**: Structured environments, parking lots

### RRT*
- **Pros**: Handles complex geometries, asymptotically optimal
- **Cons**: Slower convergence, non-deterministic
- **Best for**: Complex obstacle environments, narrow passages

## Safety Features

### Speed Guard System
Monitors trajectory safety in real-time:
- **Speed Limiting**: Enforces maximum velocity constraints
- **Acceleration Limiting**: Prevents excessive acceleration/deceleration
- **Lateral Acceleration**: Controls cornering speeds
- **Emergency Stop**: Triggers safety stop when limits exceeded

### Velocity Ramping
Applies smooth velocity profiles:
- **Acceleration Control**: Gradual speed increases
- **Deceleration Control**: Smooth speed reductions
- **Reverse Gate**: Prevents immediate direction changes at speed

### Course-Out Detection
Monitors trajectory adherence:
- **Distance Threshold**: Detects when vehicle deviates from path
- **Automatic Replanning**: Triggers new trajectory generation
- **Safety Margins**: Configurable tolerance levels

## Configuration Examples

### High-Speed Configuration
```yaml
cruise_velocity: 2.78  # 10 km/h
speed_guard.max_speed_mps: 3.0
ramp.max_accel_mps2: 1.5
algorithm_name: "astar"
```

### Precision Maneuvering
```yaml
cruise_velocity: 1.0   # Slow and precise
speed_guard.max_speed_mps: 1.2
ramp.max_accel_mps2: 0.5
algorithm_name: "rrtstar"
th_arrived_distance_m: 0.5
```

### No-Driving Zone Setup
```yaml
ndz.enable: true
ndz.osm_path: "/path/to/map.osm"
ndz.way_id: -10000
ndz.ref_lat: 37.1234
ndz.ref_lon: 127.5678
```

## Troubleshooting

### Common Issues

1. **No trajectory generated**
   - Check occupancy grid topic is publishing
   - Verify goal pose is reachable
   - Ensure sufficient free space around start/goal

2. **Slow planning performance**
   - Reduce `time_limit` parameter
   - Try switching algorithm (`astar` vs `rrtstar`)
   - Optimize costmap resolution

3. **Jerky motion**
   - Enable velocity ramping: `ramp.enable: true`
   - Reduce acceleration limits
   - Adjust speed guard parameters

4. **NDZ not working**
   - Verify OSM file path and format
   - Check way_id exists in OSM data
   - Ensure coordinate reference parameters are correct

## License

TODO: License declaration

## Maintainer

sky (minhyeok.bang@skyautonet.com)
