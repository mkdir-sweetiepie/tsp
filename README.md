#!/bin/bash

# Script to generate README.md for the TSP ROS2 package

cat > README.md << 'EOF'
# TSP (Traveling Salesman Problem) Solver for ROS2

![TSP Solver Visualization](https://github.com/user-attachments/assets/a1002c61-1530-4454-a97f-fd761a2992cd)

## Overview

This package implements a Traveling Salesman Problem (TSP) solver with visualization capabilities for ROS2. It provides an efficient algorithm to find the shortest possible route that visits a set of points exactly once and returns to the origin point.

## Features

- Fast TSP optimization algorithms
- Real-time 3D visualization of routes and solutions
- Integration with ROS2 ecosystem
- Support for dynamic point updates
- Multiple solver options (nearest neighbor, genetic algorithm, etc.)

## Dependencies

- ROS2 (tested on Humble and Foxy)
- Qt5 Data Visualization library
- C++14 or higher

## Installation

### 1. Install required dependencies

```bash
sudo apt update
sudo apt install libqt5datavisualization5-dev
```

### 2. Clone the repository into your ROS2 workspace

```bash
cd ~/ros2_ws/src/
git clone https://github.com/yourusername/tsp.git
```

### 3. Build the package

```bash
cd ~/ros2_ws/
colcon build --packages-select tsp
source install/setup.bash
```

## Usage

### Basic Execution

To run the TSP solver with visualization:

```bash
ros2 run tsp tsp
```

### Configuration

You can configure the solver by modifying the parameters in the config file:

```bash
ros2 run tsp tsp --ros-args -p num_points:=20 -p algorithm:=genetic
```

Available parameters:
- `num_points`: Number of points to generate (default: 10)
- `algorithm`: Solver algorithm (options: nearest, genetic, simulated_annealing, default: genetic)
- `display_mode`: Visualization mode (options: 2d, 3d, default: 3d)

## Topics

### Published Topics
- `/tsp/path` (geometry_msgs/Path): The calculated optimal path
- `/tsp/points` (geometry_msgs/PoseArray): The set of points being processed

### Subscribed Topics
- `/tsp/add_point` (geometry_msgs/PoseStamped): Add a new point to the current set
- `/tsp/reset` (std_msgs/Empty): Reset all points and start over

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

For any questions or suggestions, please open an issue on t
