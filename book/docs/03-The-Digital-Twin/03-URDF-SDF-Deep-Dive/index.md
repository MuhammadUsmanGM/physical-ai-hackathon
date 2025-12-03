---
id: urdf-sdf-deep-dive
title: URDF and SDF Robot Description (In-Depth)
slug: /module-03/urdf-sdf-deep-dive
---

# URDF and SDF Robot Description (In-Depth)

## Introduction

**URDF** (Unified Robot Description Format) and **SDF** (Simulation Description Format) are XML-based formats for describing robots and environments. While URDF is ROS-specific, SDF is Gazebo's native format with more advanced features. This chapter provides an in-depth comparison and advanced usage.

## URDF vs SDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| **Origin** | ROS | Gazebo |
| **Structure** | Tree only | Tree or graph |
| **Closed loops** | ❌ No | ✅ Yes |
| **Multiple robots** | ❌ No | ✅ Yes |
| **Plugins** | Limited | Extensive |
| **Sensors** | Basic | Advanced |
| **Physics** | Basic | Advanced |
| **Versioning** | No | Yes (SDF 1.x) |

---

## Advanced URDF Techniques

### 1. Xacro Macros for Complex Robots

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Properties -->
  <xacro:property name="pi" value="3.14159"/>
  <xacro:property name="arm_length" value="0.3"/>
  <xacro:property name="leg_length" value="0.4"/>
  
  <!-- Macro for a complete arm -->
  <xacro:macro name="arm" params="prefix reflect">
    <!-- Upper arm -->
    <link name="${prefix}_upper_arm">
      <visual>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="${arm_length}"/>
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
      
      <collision>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="${arm_length}"/>
        </geometry>
      </collision>
      
      <xacro:inertial_cylinder mass="1.5" radius="0.04" length="${arm_length}"/>
    </link>
    
    <!-- Shoulder joint -->
    <joint name="${prefix}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0.0 ${reflect*0.15} 0.4" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="50.0" velocity="2.0" lower="${-pi/2}" upper="${pi/2}"/>
      <dynamics damping="0.7" friction="0.1"/>
    </joint>
    
    <!-- Forearm -->
    <link name="${prefix}_forearm">
      <visual>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.03" length="${arm_length}"/>
        </geometry>
        <material name="gray"/>
      </visual>
      
      <collision>
        <origin xyz="0 0 ${-arm_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.03" length="${arm_length}"/>
        </geometry>
      </collision>
      
      <xacro:inertial_cylinder mass="1.0" radius="0.03" length="${arm_length}"/>
    </link>
    
    <!-- Elbow joint -->
    <joint name="${prefix}_elbow_joint" type="revolute">
      <parent link="${prefix}_upper_arm"/>
      <child link="${prefix}_forearm"/>
      <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="30.0" velocity="2.0" lower="0.0" upper="${pi*0.8}"/>
      <dynamics damping="0.5" friction="0.1"/>
    </joint>
    
    <!-- Hand -->
    <xacro:hand prefix="${prefix}"/>
  </xacro:macro>
  
  <!-- Macro for inertial properties -->
  <xacro:macro name="inertial_cylinder" params="mass radius length">
    <inertial>
      <mass value="${mass}"/>
      <inertia
        ixx="${mass*(3*radius*radius + length*length)/12}"
        ixy="0.0" ixz="0.0"
        iyy="${mass*(3*radius*radius + length*length)/12}"
        iyz="0.0"
        izz="${mass*radius*radius/2}"/>
    </inertial>
  </xacro:macro>
  
  <!-- Macro for hand with fingers -->
  <xacro:macro name="hand" params="prefix">
    <link name="${prefix}_hand">
      <visual>
        <geometry>
          <box size="0.08 0.06 0.1"/>
        </geometry>
        <material name="skin">
          <color rgba="0.9 0.7 0.6 1.0"/>
        </material>
      </visual>
      
      <collision>
        <geometry>
          <box size="0.08 0.06 0.1"/>
        </geometry>
      </collision>
      
      <inertial>
        <mass value="0.3"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_wrist_joint" type="fixed">
      <parent link="${prefix}_forearm"/>
      <child link="${prefix}_hand"/>
      <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
    </joint>
    
    <!-- Add fingers -->
    <xacro:finger prefix="${prefix}" finger_name="thumb" position="0.03 0 0"/>
    <xacro:finger prefix="${prefix}" finger_name="index" position="0.04 0.02 0"/>
    <xacro:finger prefix="${prefix}" finger_name="middle" position="0.04 0 0"/>
  </xacro:macro>
  
  <!-- Macro for finger -->
  <xacro:macro name="finger" params="prefix finger_name position">
    <link name="${prefix}_${finger_name}_finger">
      <visual>
        <geometry>
          <cylinder radius="0.008" length="0.04"/>
        </geometry>
        <material name="skin"/>
      </visual>
      
      <collision>
        <geometry>
          <cylinder radius="0.008" length="0.04"/>
        </geometry>
      </collision>
      
      <inertial>
        <mass value="0.01"/>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
    </link>
    
    <joint name="${prefix}_${finger_name}_joint" type="revolute">
      <parent link="${prefix}_hand"/>
      <child link="${prefix}_${finger_name}_finger"/>
      <origin xyz="${position}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit effort="5.0" velocity="1.0" lower="0" upper="${pi/2}"/>
    </joint>
  </xacro:macro>
  
  <!-- Use macros -->
  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>
  
</robot>
```

---

## SDF Advanced Features

### 1. Multiple Robots in One File

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="multi_robot_world">
    
    <!-- Robot 1 -->
    <model name="humanoid_1">
      <pose>0 0 1 0 0 0</pose>
      <include>
        <uri>model://humanoid_robot</uri>
      </include>
    </model>
    
    <!-- Robot 2 -->
    <model name="humanoid_2">
      <pose>2 0 1 0 0 0</pose>
      <include>
        <uri>model://humanoid_robot</uri>
      </include>
    </model>
    
    <!-- Shared environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
    
  </world>
</sdf>
```

### 2. Closed Kinematic Loops

```xml
<!-- SDF supports closed loops (URDF doesn't) -->
<model name="parallel_mechanism">
  <link name="base"/>
  
  <link name="arm1"/>
  <joint name="joint1" type="revolute">
    <parent>base</parent>
    <child>arm1</child>
  </joint>
  
  <link name="arm2"/>
  <joint name="joint2" type="revolute">
    <parent>base</parent>
    <child>arm2</child>
  </joint>
  
  <link name="end_effector"/>
  <joint name="joint3" type="revolute">
    <parent>arm1</parent>
    <child>end_effector</child>
  </joint>
  
  <!-- Closes the loop -->
  <joint name="joint4" type="revolute">
    <parent>arm2</parent>
    <child>end_effector</child>
  </joint>
</model>
```

### 3. Advanced Sensors

```xml
<model name="humanoid_with_sensors">
  
  <!-- RGB-D Camera -->
  <link name="camera_link">
    <sensor name="rgbd_camera" type="depth">
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
          <remapping>image_raw:=rgb/image_raw</remapping>
          <remapping>depth/image_raw:=depth/image_raw</remapping>
        </ros>
      </plugin>
    </sensor>
  </link>
  
  <!-- LiDAR -->
  <link name="lidar_link">
    <sensor name="lidar" type="gpu_ray">
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </link>
  
  <!-- Force/Torque Sensor -->
  <joint name="left_ankle_joint" type="revolute">
    <parent>left_shin</parent>
    <child>left_foot</child>
    <sensor name="left_foot_force" type="force_torque">
      <update_rate>100</update_rate>
      <force_torque>
        <frame>child</frame>
        <measure_direction>child_to_parent</measure_direction>
      </force_torque>
    </sensor>
  </joint>
  
</model>
```

---

## Converting Between URDF and SDF

### URDF to SDF

```bash
# Using Gazebo tool
gz sdf -p robot.urdf > robot.sdf

# Or in Python
from sdformat_tools import urdf_to_sdf
sdf_content = urdf_to_sdf('robot.urdf')
```

### SDF to URDF (Limited)

```python
# Note: SDF to URDF loses features (closed loops, multiple robots, etc.)
from sdformat_tools import sdf_to_urdf
urdf_content = sdf_to_urdf('robot.sdf')
```

---

## Best Practices

### 1. Modular Design

```xml
<!-- Main robot file -->
<robot name="humanoid">
  <xacro:include filename="$(find robot_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find robot_description)/urdf/torso.xacro"/>
  <xacro:include filename="$(find robot_description)/urdf/arms.xacro"/>
  <xacro:include filename="$(find robot_description)/urdf/legs.xacro"/>
  <xacro:include filename="$(find robot_description)/urdf/sensors.xacro"/>
  
  <xacro:torso/>
  <xacro:arms/>
  <xacro:legs/>
  <xacro:sensors/>
</robot>
```

### 2. Parameter Files

```yaml
# robot_params.yaml
robot:
  dimensions:
    torso_height: 0.5
    torso_width: 0.3
    arm_length: 0.3
    leg_length: 0.4
  
  masses:
    torso: 15.0
    upper_arm: 1.5
    forearm: 1.0
    thigh: 5.0
    shin: 3.0
  
  limits:
    shoulder_range: [-1.57, 1.57]
    elbow_range: [0.0, 2.5]
    hip_range: [-0.5, 2.0]
    knee_range: [-2.5, 0.0]
```

Load in Xacro:

```xml
<xacro:property name="config" value="${load_yaml('$(find robot_description)/config/robot_params.yaml')}"/>
<xacro:property name="torso_height" value="${config['robot']['dimensions']['torso_height']}"/>
```

---

## Summary

- **URDF:** ROS-native, tree structure, good for most robots
- **SDF:** More advanced, supports closed loops and multiple robots
- **Xacro:** Essential for maintainable robot descriptions
- **Conversion:** Possible but may lose features
- **Best practices:** Modular design, parameter files, version control

Understanding both formats allows you to leverage the best tools for your application.

---

## Further Reading

- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [SDF Specification](http://sdformat.org/)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [Gazebo SDF Tutorial](http://gazebosim.org/tutorials?tut=build_robot)
