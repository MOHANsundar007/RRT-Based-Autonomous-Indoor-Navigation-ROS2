# ROS2 RRT Indoor Path Planning and Autonomous Navigation

<img width="1437" height="951" alt="result" src="https://github.com/user-attachments/assets/e8922974-a1ed-4d9e-8244-ba0646937647" />

---

## Overview

This project presents a complete autonomous indoor navigation framework using a custom Rapidly-exploring Random Tree (RRT) path planning algorithm integrated with the ROS2 Navigation Stack (Nav2), Gazebo simulation, and RViz visualization tools.

The system performs autonomous path planning and navigation inside indoor environments while visualizing the complete RRT tree expansion process in real time.

The project includes:

- Custom RRT path planner
- Dynamic tree visualization
- Occupancy grid collision checking
- Smoothed trajectory generation
- AMCL localization
- Gazebo simulation
- RViz visualization
- ROS2 Nav2 integration
- Differential drive robot navigation

This project is designed for robotics research, autonomous navigation experiments, and indoor service robot applications.

---

# Features

## Autonomous Navigation
- Goal based autonomous indoor navigation
- Differential drive robot support
- ROS2 Navigation Stack integration

## Custom RRT Planner
- Custom implementation of Rapidly exploring Random Tree (RRT)
- Goal bias sampling
- Collision free node expansion
- OccupancyGrid based obstacle checking

## Advanced Visualization
- Real-time RRT tree animation
- Depth based tree coloring
- Frontier visualization
- Accepted and rejected sample visualization
- Start and goal markers

## Path Optimization
- Smoothed navigation trajectory generation
- Interpolated waypoint smoothing
- Improved navigation continuity

## Simulation Environment
- Gazebo simulation support
- RViz visualization support
- Indoor map-based navigation

---

# System Architecture

<img width="1920" height="1080" alt="Untitled design (3)" src="https://github.com/user-attachments/assets/72ba3342-f574-4d74-8a78-6767550cf54f" />


---

# Technologies Used

- ROS2 Humble
- Python
- Gazebo
- RViz2
- Nav2
- AMCL Localization
- OccupancyGrid Mapping
- NumPy
---

# Installation

## Clone Repository

```bash
mkdir -p ~/robot_ws/src

cd ~/robot_ws/src

git clone https://github.com/YOUR_USERNAME/ROS2-RRT-Indoor-Path-Planning.git
```

---

## Build Workspace

```bash
cd ~/robot_ws

colcon build
```

---

## Source Workspace

```bash
source install/setup.bash
```

---

# Launch Simulation

```bash
ros2 launch path_planning_pkg navigation.launch.py
```

---

# RViz Visualization Topics

| Topic | Description |
|------|------|
| `/rrt_tree` | RRT tree visualization |
| `/rrt_path` | Raw generated RRT path |
| `/rrt_smoothed_path` | Smoothed navigation path |
| `/rrt_samples` | Accepted and rejected sample points |
| `/plan` | Final navigation plan |

---

# RRT Algorithm Pipeline

## 1. Random Sampling
The planner randomly samples nodes inside the map space.

## 2. Nearest Node Search
The nearest existing tree node is selected.

## 3. Steering
A new node is generated toward the sampled point.

## 4. Collision Checking
OccupancyGrid collision validation is performed.

## 5. Tree Expansion
Valid nodes are added to the RRT tree.

## 6. Goal Detection
Planning terminates when the goal tolerance is satisfied.

## 7. Path Smoothing
The raw path is smoothed using interpolation filtering.

---

# Visualization Features

## Dynamic Tree Animation
- Animated RRT growth
- Incremental edge rendering

## Gradient-Based Depth Coloring
- Cyan → Green → Yellow → Orange
- Visualizes tree depth progression

## Frontier Node Visualization
- Active exploration node highlighting

## Sample Classification
- Accepted samples
- Rejected samples

---

# Simulation Screenshots

## RViz Visualization

<p align="center">
  <img width="1437" height="951" alt="result" src="https://github.com/user-attachments/assets/e8922974-a1ed-4d9e-8244-ba0646937647" />
</p>

---

## Gazebo Environment

<img width="675" height="670" alt="gazebo" src="https://github.com/user-attachments/assets/bb38b470-ff06-4d42-a15b-5ecb52e71e03" />


---

# Demo


---

# Launch Components

The launch system automatically starts:

- Gazebo server
- Gazebo client
- Robot State Publisher
- Robot spawning
- Map Server
- AMCL localization
- Planner Server
- Controller Server
- Behavior Tree Navigator
- RViz2
- Custom RRT planner

---

# Future Improvements

- Dynamic obstacle avoidance
- Reinforcement learning optimization
- Real-world robot deployment


---

# Applications

- Indoor service robots
- Restaurant delivery robots
- Hospital delivery robots
- Autonomous warehouse robots
- Research and education
- Autonomous mobile robotics

---

# Research Contribution

This project demonstrates the integration of custom sampling based path planning algorithms with ROS2 autonomous navigation systems and real time visualization frameworks.

The system can serve as a foundation for future research in:

- Intelligent robotics
- Autonomous navigation
- AI-based planning
- Hybrid optimization algorithms
- Mobile robot systems

---

# Author

MohanaSundaram G

Robotics and AI Developer

---

# License

This project is licensed under the MIT License.

---

# Acknowledgements

- ROS2
- Nav2
- Gazebo
- RViz2
- Open Robotics
- ROS Community

---
