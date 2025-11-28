# Trajectory Smoothening & Tracking System for Differential Drive Robots

## Overview
Complete implementation of path smoothing, trajectory generation, and trajectory tracking controller for Turtlebot3 simulation.

## Features
- **Path Smoothing**: Cubic spline interpolation for smooth paths
- **Trajectory Generation**: Trapezoidal velocity profile
- **Controller**: Pure Pursuit algorithm for trajectory tracking
- **Visualization**: RViz integration with path and waypoint markers

## Path Comparison Visualization
The following plot illustrates the comparison between the original discrete waypoints (red), the smoothed continuous path (blue). This demonstrates the effectiveness of cubic spline smoothing in eliminating sharp corners while maintaining waypoint fidelity.

![Path Comparison: Original Waypoints vs. Smoothed Path vs. Actual Trajectory](/src/trajectory_tracking/trajectory_tracking/path_comparison.png)

## Installation

### Prerequisites
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+

### Setup
```bash
# Install dependencies
sudo apt install ros-humble-turtlebot3* python3-pip
pip3 install numpy scipy matplotlib

# Build workspace
cd ~/robot_ws
colcon build --packages-select trajectory_tracking
source install/setup.bash
```

## Usage

### Run Simulation
```bash
# Terminal 1: Source and set environment
source ~/robot_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger

# Launch everything
ros2 launch trajectory_tracking trajectory_simulation.launch.py
```

### Generate Analysis Plots
```bash
# Run after simulation to produce trajectory_analysis.png
python3 ~/robot_ws/src/trajectory_tracking/analysis_plotting.py
```

## Architecture

### Modules
1. **PathSmoother**: Converts discrete waypoints to smooth continuous path
2. **TrajectoryGenerator**: Adds time parameterization and velocity profiles
3. **PurePursuitController**: Tracks generated trajectory
4. **TrajectoryTrackingNode**: Main ROS2 node integrating all components

### Design Decisions
- **Cubic Splines**: Chosen for C² continuity (smooth acceleration) and best for robotics appilcations
- **Pure Pursuit**: Simple, effective, well-suited for differential drive
- **Trapezoidal Profile**: Respects velocity and acceleration limits

## Extending to Real Robot

### Requirements
1. Hardware: Turtlebot3, LIDAR, IMU, wheel encoders
2. Localization: AMCL or EKF for pose estimation
3. Mapping: SLAM for environment map
4. Safety: Emergency stop, collision avoidance

### Modifications
- Replace `/odom` with filtered pose from `robot_localization`
- Add sensor fusion (IMU + encoders + visual odometry)
- Implement obstacle detection using LIDAR

## Obstacle Avoidance (Extra Credit)

### Approach
1. Subscribe to `/scan` topic for LIDAR data
2. Implement Dynamic Window Approach (DWA) for local planning
3. Modify trajectory in real-time to avoid obstacles (set a min. dstance from the object)

## AI Tools Used
- **Claude AI**: Architecture design, algorithm selection
- **GitHub Copilot**: Code completion and boilerplate

## Some Parameters
- Cross-track error: < 0.1m
- Goal tolerance: 0.1m
- Max velocity: 0.3 m/s
- Control frequency: 10 Hz
