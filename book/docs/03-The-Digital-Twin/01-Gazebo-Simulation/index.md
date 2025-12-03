---
id: gazebo-simulation
title: Gazebo Simulation Environment
slug: /module-03/gazebo-simulation
---

# Gazebo Simulation Environment

Gazebo, now part of the Ignition suite, is the leading open-source robotics simulator. It provides accurate physics simulation, realistic sensor simulation, and seamless integration with ROS 2, making it the ideal platform for testing and developing robot algorithms.

## Setting Up Gazebo with ROS 2

Gazebo integrates with ROS 2 through the `ros_gz` package, which provides bridges between ROS 2 messages and Gazebo topics. The integration enables:

- **Synchronous Control**: ROS 2 nodes can control robots in Gazebo in real-time
- **Sensor Data**: Robot sensors in Gazebo publish data to ROS 2 topics
- **Physics Simulation**: Gazebo's physics engine handles robot-environment interactions

**Installation and Setup**:
```bash
# Install Gazebo Garden (or Ignition Fortress)
sudo apt install ros-humble-ros-gz

# Launch Gazebo with ROS 2 bridge
ros2 launch ros_gz_sim sim_world.launch.py world_name:=empty.sdf
```

## Robot Description with URDF and SDF

Gazebo supports two primary formats for robot description:

**URDF (Unified Robot Description Format)**: XML-based format used by ROS for representing robot structure. URDF defines:
- Kinematic structure (joints and links)
- Visual and collision properties
- Inertial properties
- Sensor placements

**SDF (Simulation Description Format)**: XML format specific to Gazebo that extends URDF capabilities with:
- Gazebo-specific plugins
- Physics parameters
- Simulation-specific properties

## Sensor Simulation in Gazebo

Gazebo provides realistic simulation of various sensors, crucial for developing perception systems:

**Camera Simulation**:
- RGB cameras for computer vision
- Depth cameras for 3D perception
- Stereo cameras for depth estimation
- Parameters include resolution, field of view, and noise models

**LiDAR Simulation**:
- 2D and 3D LiDAR sensors
- Configurable parameters: range, resolution, noise
- Used for mapping and navigation algorithms

**IMU Simulation**:
- Accelerometer, gyroscope, and magnetometer
- Realistic noise models and drift characteristics
- Essential for robot localization and control