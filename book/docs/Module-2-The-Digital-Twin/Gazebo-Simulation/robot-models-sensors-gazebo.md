---
id: robot-models-sensors-gazebo
title: Robot Models and Sensor Simulation in Gazebo
sidebar_label: Robot Models and Sensors
---

# Robot Models and Sensor Simulation in Gazebo

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

**URDF Example**:
```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="1" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Creating and Importing Robot Models

Creating complex robot models in Gazebo involves several steps:

1. **Design in CAD Software**: Create the 3D model in CAD software (SolidWorks, Fusion 360, etc.)
2. **Export to URDF**: Use software like `sw_urdf_exporter` to convert CAD models to URDF
3. **Add Simulation Properties**: Enhance URDF with Gazebo-specific properties like material colors, plugins
4. **Test in Simulation**: Load the model into Gazebo to verify it works correctly

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

**Force/Torque Sensors**:
- Simulated force and torque measurements
- Critical for manipulation tasks

## Gazebo Plugins and ROS 2 Integration

Gazebo uses plugins to extend functionality. Key plugins for ROS 2 integration include:

**ros_gz_bridge**: Translates between ROS 2 messages and Gazebo transport
**gz::sim::systems::DiffDrive**: Differential drive controller plugin
**gz::sim::systems::JointPositionController**: Joint control plugin
**gz::sim::systems::Imu**: IMU sensor plugin