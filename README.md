# BirdRo Planner

A ROS 2 package that provides free-space based path planning capabilities for autonomous vehicles, specifically designed for parking and low-speed maneuvering scenarios.

## Overview

BirdRo Planner is a sophisticated path planning system that uses occupancy grid maps to generate safe trajectories in free space. The package supports multiple planning algorithms and includes advanced features like speed control, obstacle avoidance, and No Driving Zone (NDZ) handling.

### Key Features

- **Multiple Planning Algorithms**: Supports both A* and RRT* algorithms for path planning
- **Free-Space Planning**: Optimized for parking and low-speed maneuvering scenarios
- **Speed Control**: Built-in speed ramping and acceleration limiting
- **Obstacle Avoidance**: Real-time replanning when obstacles are detected
- **No Driving Zone Support**: Integration with OSM-based restricted areas
- **Safety Features**: Speed guard system with configurable limits
- **ROS 2 Integration**: Full ROS 2 compatibility with standard message types

## Architecture

The package consists of two main components:

1. **BirdRo Planner Node** (`birdro_planner_node`): Main planning node that generates trajectories
2. **NDZ Costmap Overlay** (`ndz_costmap_overlay`): Overlays No Driving Zones onto occupancy grids

## Dependencies

This package requires the following ROS 2 dependencies:

- `rclcpp` - ROS 2 C++ client library
- `rclcpp_components` - ROS 2 component system
- `autoware_freespace_planning_algorithms` - Core planning algorithms
- `autoware_planning_msgs` - Planning message definitions
- `autoware_motion_utils` - Motion utilities
- `autoware_universe_utils` - General utilities
- `autoware_map_msgs` - Map message definitions
- `autoware_lanelet2_extension` - Lanelet2 extensions
- `tinyxml2_vendor` - XML parsing for OSM files
- `tier4_map_msgs` - Map projection messages
- `geography_utils` - Geographic utilities
- `lanelet2_*` - Lanelet2 mapping framework

## Installation

### Prerequisites

Ensure you have ROS 2 and the required dependencies installed:

```bash
# Install ROS 2 (Humble/Iron/Rolling)
# Follow official ROS 2 installation guide

# Install dependencies using rosdep
rosdep install --from-paths . --ignore-src -r -y
```

### Building

```bash
# In your ROS 2 workspace
colcon build --packages-select birdro_planner

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Launch

Launch the complete planner system:

```bash
ros2 launch birdro_planner birdro_planner.launch.xml
```

### Individual Nodes

Launch individual components:

```bash
# Main planner node
ros2 run birdro_planner birdro_planner_node

# NDZ costmap overlay
ros2 run birdro_planner ndz_costmap_overlay
```

## Configuration

### Planning Parameters

Key configuration parameters for the planner:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `algorithm_name` | "astar" | Planning algorithm ("astar" or "rrtstar") |
| `cruise_velocity` | 1.39 m/s | Default cruise velocity |
| `update_rate` | 10.0 Hz | Planning update frequency |
| `th_arrived_distance_m` | 1.0 m | Distance threshold for goal arrival |
| `obstacle_threshold` | 100 | Occupancy grid threshold for obstacles |
| `time_limit` | 30000.0 ms | Maximum planning time |

### Speed Control Parameters

Configure speed limits and acceleration:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed_guard.enable` | true | Enable speed guard system |
| `speed_guard.max_speed_mps` | 1.5 m/s | Maximum allowed speed |
| `speed_guard.max_accel_mps2` | 2.5 m/s² | Maximum acceleration |
| `ramp.max_accel_mps2` | 1.0 m/s² | Speed ramp acceleration limit |
| `ramp.max_decel_mps2` | 1.0 m/s² | Speed ramp deceleration limit |

### NDZ (No Driving Zone) Parameters

Configure restricted areas:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ndz.enable` | true | Enable NDZ processing |
| `ndz.osm_path` | "" | Path to OSM file with NDZ data |
| `ndz.way_id` | -10000 | OSM way ID for NDZ polygon |

## Topics

### Subscribed Topics

- `/planning/scenario_planning/parking/costmap_generator/occupancy_grid_ndz` - Occupancy grid with NDZ overlay
- `/localization/kinematic_state` - Vehicle odometry
- `/map/vector_map` - Vector map data
- `/planning/mission_planning/goal` - Goal pose

### Published Topics

- `/planning/scenario_planning/parking/trajectory` - Generated trajectory
- `/planning/scenario_planning/parking/is_completed` - Planning completion status
- Various debug and visualization topics

## Algorithm Details

### A* Planning
- Grid-based search with configurable resolution
- Supports forward/reverse driving
- Considers vehicle kinematics and turning constraints

### RRT* Planning
- Sampling-based planning for complex scenarios
- Asymptotically optimal solutions
- Better performance in high-dimensional spaces

### Speed Profile Generation
- Acceleration/deceleration limiting
- Smooth velocity transitions
- Reverse gate functionality for direction changes

## Safety Features

### Speed Guard System
- Real-time speed and acceleration monitoring
- Automatic trajectory modification for safety
- Configurable violation thresholds and responses

### Obstacle Detection and Replanning
- Continuous monitoring of the planned path
- Automatic replanning when obstacles are detected
- Debouncing to prevent excessive replanning

## Development

### Code Structure

```
├── include/
│   └── birdro_planner.hpp          # Main planner class definition
├── src/
│   ├── birdro_planner.cpp          # Main planner implementation
│   └── ndz_costmap_overlay.cpp     # NDZ overlay implementation
├── launch/
│   └── birdro_planner.launch.xml   # Launch configuration
├── CMakeLists.txt                  # Build configuration
└── package.xml                     # Package metadata
```

### Building for Development

```bash
# Build with debug symbols
colcon build --packages-select birdro_planner --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run tests (if available)
colcon test --packages-select birdro_planner
```

## Examples

### Setting a Goal Pose

```bash
# Publish a goal pose
ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 10.0, y: 5.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

### Monitoring Planning Status

```bash
# Monitor trajectory output
ros2 topic echo /planning/scenario_planning/parking/trajectory

# Check completion status
ros2 topic echo /planning/scenario_planning/parking/is_completed
```

## Troubleshooting

### Common Issues

1. **No trajectory generated**: Check occupancy grid input and goal pose validity
2. **Planning timeout**: Increase `time_limit` parameter or simplify the scenario
3. **Excessive replanning**: Adjust `obstacle_replan_confirm_count` and interval parameters
4. **Speed violations**: Review speed guard parameters and vehicle dynamics

### Debug Topics

Enable debug visualization:
- Set `visualize_goal_marker: true` for goal visualization
- Set `use_occlusion_marker: true` for occlusion debugging

## License

TODO: License declaration

## Maintainer

- sky (minhyeok.bang@skyautonet.com)

## Contributing

Please follow the existing code style and ensure all changes are tested before submitting pull requests.
