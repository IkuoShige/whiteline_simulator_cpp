# Whiteline Simulator C++ for Soccer Field

This ROS2 package simulates white line detection and landmark recognition from a robot's perspective on an AdultSize soccer field. The package generates detailed white line points representing the soccer field boundaries, goal areas, penalty areas, center circle, and penalty marks, as well as landmark detection for field intersections.

## Features

- **Soccer Field White Line Generation**: Automatically generates white line points for an AdultSize soccer field (14m x 9m)
- **Landmark Detection**: Detects and classifies T-junctions, L-corners, X-crosses, and penalty marks
- **Real-time Robot Simulation**: Simulates a robot moving on the field with configurable field of view (FOV) and detection range
- **ROS2 Integration**: Publishes white line points, landmarks, robot pose, and visualization markers
- **Configurable Parameters**: Adjustable field dimensions, point spacing, FOV, and detection parameters
- **Visualization**: Full rviz2 support with markers for field and landmark visualization

## Soccer Field Dimensions (AdultSize)

- **Field Length**: 14m
- **Field Width**: 9m  
- **Goal Depth**: 0.6m
- **Goal Width**: 2.6m
- **Goal Area**: 1m x 4m
- **Penalty Area**: 3m x 6m
- **Center Circle**: 3m diameter
- **Penalty Mark Distance**: 2.1m from goal line

## Landmark Types

- **T_JUNCTION** (Red): T-shaped intersections where penalty areas meet field boundaries
- **L_CORNER** (Blue): L-shaped corners at field boundaries, penalty areas, and goal areas
- **X_CROSS** (Yellow): X-shaped intersections at the center of the field and center circle intersections
- **PENALTY_MARK** (Magenta): Penalty marks at 2.1m from each goal line
- **GOAL_POST** (Cyan): Goal posts at the intersection of goal lines and goal boundaries

## Package Structure

```
whiteline_simulator_cpp/
├── src/
│   ├── whiteline_simulator_node.cpp    # Main simulator node
│   ├── whiteline_listener.cpp          # Example listener node
│   ├── soccer_field_generator.cpp      # Field generation utility
│   └── preprocessor.cpp                # Core processing library
├── include/
│   └── whiteline_simulator_cpp/
│       ├── preprocessor.hpp            # Library header
│       └── pose.hpp                    # Pose structure
├── config/
│   └── soccer_field_visualization.rviz # RViz configuration
└── data/
    ├── soccer_field_points.txt         # Generated field data
    └── soccer_field_landmarks.txt      # Generated landmark data
```

## Building

Make sure you have ROS2 Humble installed with pixi environment management:

```bash
cd /path/to/your/ros2_workspace
pixi run colcon build --packages-select whiteline_simulator_cpp
```

## Usage

### 1. Generate Soccer Field Data

Generate the soccer field white line points and landmarks:

```bash
pixi run ./install/whiteline_simulator_cpp/lib/whiteline_simulator_cpp/soccer_field_generator src/whiteline_simulator_cpp/data/soccer_field_points.txt 0.05 src/whiteline_simulator_cpp/data/soccer_field_landmarks.txt
```

Parameters:
- `output_file`: Path to white line points output file
- `point_spacing`: Spacing between points in meters (default: 0.05m)
- `landmarks_file`: Path to landmarks output file

### 2. Run the Simulator

Start the white line and landmark simulator:

```bash
pixi run ros2 run whiteline_simulator_cpp whiteline_simulator_node --ros-args -p whiteline_points_file:=src/whiteline_simulator_cpp/data/soccer_field_points.txt -p landmarks_file:=src/whiteline_simulator_cpp/data/soccer_field_landmarks.txt -p generate_soccer_field:=false
```

### 3. Run the Listener (Optional)

In another terminal, run the listener to see the detected white line points and landmarks:

```bash
pixi run ros2 run whiteline_simulator_cpp whiteline_listener
```

### 4. Visualize with RViz2

Launch RViz2 with the provided configuration:

```bash
pixi run rviz2 -d src/whiteline_simulator_cpp/config/soccer_field_visualization.rviz
```

## Parameters

The simulator node accepts the following parameters:

- `whiteline_points_file`: Path to white line points file (default: "soccer_field_points.txt")
- `landmarks_file`: Path to landmarks file (default: "soccer_field_landmarks.txt")
- `generate_soccer_field`: Whether to generate soccer field on startup (default: true)
- `point_spacing`: Spacing between generated points in meters (default: 0.05)
- `fov`: Field of view in radians (default: π/2)
- `max_distance`: Maximum detection distance in meters (default: 8.0)
- `robot_speed`: Robot movement speed (default: 0.1)
- `field_length`: Soccer field length in meters (default: 14.0)
- `field_width`: Soccer field width in meters (default: 9.0)

## Published Topics

- `/whiteline_points` (std_msgs/Float32MultiArray): Detected white line points
- `/landmarks` (std_msgs/Float32MultiArray): Detected landmarks with types and orientations
- `/robot_pose` (geometry_msgs/Pose2D): Current robot pose
- `/whiteline_markers` (visualization_msgs/MarkerArray): Visualization markers for RViz2

## Example Usage with Custom Parameters

```bash
pixi run ros2 run whiteline_simulator_cpp whiteline_simulator_node --ros-args \
  -p fov:=1.57 \
  -p max_distance:=5.0 \
  -p robot_speed:=0.2 \
  -p point_spacing:=0.03
```

## Data Format

### White Line Points
The white line points are published as a Float32MultiArray with the following format:
- Layout: [x1, y1, x2, y2, x3, y3, ...]
- Coordinates are in meters relative to the field center (0, 0)

### Landmarks
The landmarks are published as a Float32MultiArray with the following format:
- Layout: [x1, y1, type1, orientation1, x2, y2, type2, orientation2, ...]
- Coordinates are in meters relative to the field center (0, 0)
- Types: 0=T_JUNCTION, 1=L_CORNER, 2=X_CROSS, 3=PENALTY_MARK, 4=GOAL_POST
- Orientation in radians

## Visualization Features

The RViz2 visualization includes:
- **Gray dots**: All soccer field white line points
- **White dots**: Currently visible white line points
- **Red cubes**: T-junction landmarks (all and visible)
- **Blue cubes**: L-corner landmarks (all and visible)
- **Yellow cubes**: X-cross landmarks (all and visible)
- **Magenta cubes**: Penalty mark landmarks (all and visible)
- **Cyan cubes**: Goal post landmarks (all and visible)
- **Red arrow**: Robot pose and orientation
- **Green triangle**: Robot's field of view (FOV)

## Landmark Detection

The system automatically detects and classifies the following landmarks:

1. **T-Junctions**: 
   - Penalty area boundaries meeting field boundaries
   - Goal area boundaries meeting penalty area boundaries
   - Center line meeting field boundaries

2. **L-Corners**:
   - Field corners (4 corners)
   - Penalty area corners (4 corners per penalty area)
   - Goal area corners (4 corners per goal area)

3. **X-Crosses**:
   - Center point of the field
   - Center circle and center line intersections (2 points)

4. **Penalty Marks**:
   - Left and right penalty marks at 2.1m from goal lines

5. **Goal Posts**:
   - Intersections of goal lines and goal boundaries (4 posts total)

## License

MIT License - See LICENSE file for details

## Contributing

Feel free to submit issues and enhancement requests! 