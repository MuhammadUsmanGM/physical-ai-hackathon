---
id: robot-models-sensors-gazebo
title: Robot Models and Sensor Simulation in Gazebo
sidebar_label: Robot Models and Sensors
---

# Robot Models and Sensor Simulation in Gazebo

Creating accurate robot models is the first step in building a Digital Twin. This chapter covers the Unified Robot Description Format (URDF), Simulation Description Format (SDF), and how to simulate sensors like Cameras, LiDARs, and IMUs.

## URDF vs. SDF

| Feature | URDF (Unified Robot Description Format) | SDF (Simulation Description Format) |
|---------|-----------------------------------------|-------------------------------------|
| **Purpose** | Standard ROS robot description | Gazebo world & robot description |
| **Structure** | Tree structure (Parent-Child) | Graph structure (Loop closures allowed) |
| **Scope** | Single robot only | Entire worlds, lighting, multiple robots |
| **Usage** | RViz visualization, MoveIt! planning | Gazebo simulation |

**Workflow:**
1. Create robot in **URDF** (for ROS compatibility).
2. Gazebo automatically converts URDF to **SDF** at runtime.

## Building a Robot Model (URDF)

A robot is a collection of **Links** (rigid bodies) connected by **Joints** (constraints).

### 1. The Base Link
Every robot needs a root link.

```xml
<?xml version="1.0"?>
<robot name="my_bot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

</robot>
```

### 2. Adding Wheels (Joints)
Connect wheels to the base using continuous joints.

```xml
  <!-- Right Wheel Link -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5707 0 0"/> <!-- Rotate cylinder -->
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
      <origin rpy="1.5707 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting Base to Wheel -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 0"/>
    <axis xyz="0 1 0"/> <!-- Rotate around Y axis -->
  </joint>
```

### 3. Gazebo-Specific Tags
To make the URDF work in Gazebo, we need `<gazebo>` tags.

```xml
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1> <!-- Friction coefficient -->
    <mu2>1.0</mu2>
  </gazebo>
```

## Simulating Sensors

Sensors are added as links and joints, with a Gazebo plugin to generate data.

### 1. Adding a LiDAR
First, add the visual link for the LiDAR.

```xml
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>
```

Then, add the Gazebo sensor plugin:

```xml
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <min_angle>-3.14</min_angle>
            <max_angle>3.14</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.3</min>
          <max>12</max>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>
```

### 2. Adding a Camera

```xml
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>my_bot/camera1</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
```

### 3. Adding an IMU

```xml
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <remapping>~/out:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
```

## Controlling the Robot (Plugins)

To move the robot, we need a controller plugin. For a 2-wheeled robot, we use the **Differential Drive** plugin.

```xml
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/demo</namespace>
      </ros>

      <!-- Wheel Joints -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <!-- Kinematics -->
      <wheel_separation>0.35</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>

      <!-- Limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <!-- Output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
```

## Domain Randomization

To make your AI robust, you should randomize physical properties during training.

**Example: Randomizing Friction in SDF**
You can write a script to generate world files with varying parameters.

```python
import random

def generate_random_physics():
    friction = random.uniform(0.5, 1.5)
    mass = random.uniform(4.0, 6.0)
    
    sdf_template = f"""
    <model name="box">
      <link name="link">
        <inertial>
          <mass>{mass}</mass>
        </inertial>
        <collision name="collision">
          <surface>
            <friction>
              <ode>
                <mu>{friction}</mu>
                <mu2>{friction}</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>
    """
    return sdf_template
```

## Visualizing in RViz

1. Launch Gazebo and spawn the robot.
2. Run `ros2 run rviz2 rviz2`.
3. Set **Fixed Frame** to `odom`.
4. Add **RobotModel** display.
5. Add **LaserScan** display (topic: `/scan`).
6. Add **Image** display (topic: `/camera/image_raw`).

## Summary

You have learned how to:
- ✓ Create a robot model using **URDF**.
- ✓ Add **Sensors** (Lidar, Camera, IMU) to the model.
- ✓ Use **Plugins** to control the robot and publish sensor data.
- ✓ Visualize the "Digital Twin" in **RViz**.

---

**Next:** Move to high-fidelity simulation with [Introduction to Unity Simulation](../Unity-Simulation/introduction-to-unity.md).