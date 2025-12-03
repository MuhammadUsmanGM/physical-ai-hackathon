---
id: introduction-to-gazebo
title: Introduction to Robotic Simulation with Gazebo
sidebar_label: Introduction to Gazebo
---

# Introduction to Robotic Simulation with Gazebo

Gazebo is the standard simulation tool for ROS-based robotics. It allows you to simulate your robot's physical interactions with the world, test algorithms, and visualize sensor data in a safe, controlled environment.

## Gazebo Classic vs. Gazebo (Ignition)

There are currently two major versions of Gazebo:

| Feature | Gazebo Classic (v11) | Gazebo (Ignition) |
|---------|----------------------|-------------------|
| **Architecture** | Monolithic | Modular (Client-Server) |
| **Rendering** | OGRE 1.x | OGRE 2.x (Physically Based Rendering) |
| **Physics** | ODE (default), Bullet | Dart, Bullet, TPE |
| **ROS Bridge** | `gazebo_ros_pkgs` | `ros_gz_bridge` |
| **Status** | Stable, End-of-Life (2025) | Future Standard |

**For this course, we will use Gazebo Classic (v11)** as it is the default for ROS 2 Humble and has the most extensive documentation and plugin support.

## Installation

### Step 1: Install Gazebo 11

Gazebo 11 is usually installed automatically with `ros-humble-desktop`, but you can ensure you have all necessary packages:

```bash
sudo apt update
sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs
```

### Step 2: Verify Installation

Run Gazebo from the terminal:

```bash
gazebo --verbose
```

You should see the Gazebo GUI open with an empty world.

## The Gazebo Interface

The Gazebo GUI consists of several key areas:

1. **Scene View (Center)**: The 3D view of your simulation.
   - **Left Click**: Select objects
   - **Right Click**: Context menu (move, rotate, delete)
   - **Scroll**: Zoom
   - **Shift + Left Click**: Rotate camera

2. **World Panel (Left)**:
   - **World Tab**: Lists all models (ground plane, sun, robots) in the scene.
   - **Insert Tab**: Library of models you can drag and drop into the world.
   - **Layers Tab**: Manage visibility of model groups.

3. **Toolbar (Top)**:
   - **Select/Translate/Rotate/Scale**: Tools for manipulating objects.
   - **Snap**: Snap objects to grid.
   - **Copy/Paste**: Duplicate models.

4. **Simulation Control (Bottom)**:
   - **Play/Pause**: Start or stop physics.
   - **Real Time Factor**: Speed of simulation vs. real time (1.0 = real time).
   - **Sim Time**: Total elapsed simulation time.

## Building Your First World

Let's create a simple test environment.

### 1. Add Objects
1. Go to the **Insert** tab on the left.
2. Find **Simple Shapes** (Box, Sphere, Cylinder).
3. Drag a **Box** into the scene.
4. Use the **Scale Tool** (top toolbar) to stretch it into a wall.
5. Add a **Cylinder** and **Sphere** as obstacles.

### 2. Add a Light Source
1. The default world has a sun.
2. You can add more lights from the toolbar (Point, Spot, Directional).

### 3. Save the World
1. File -> Save World As.
2. Save as `my_first_world.world`.

**World File Structure (.world)**:
Gazebo worlds are XML files (SDF format).

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Your custom box -->
    <model name="my_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Launching Gazebo with ROS 2

To use Gazebo with ROS 2, we use launch files.

Create a launch file `gazebo_world.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])
```

**Key Plugins:**
- `libgazebo_ros_init.so`: Initializes ROS 2 context in Gazebo.
- `libgazebo_ros_factory.so`: Allows spawning robots from ROS 2.

## Physics Engines

Gazebo supports multiple physics engines. The default is **ODE (Open Dynamics Engine)**.

| Engine | Characteristics | Best For |
|--------|-----------------|----------|
| **ODE** | Stable, good performance, default | General robotics, mobile bases |
| **Bullet** | Better collision detection | Manipulation, stacking objects |
| **Dart** | High accuracy for kinematic chains | Humanoids, complex linkages |
| **Simbody** | Biomechanical accuracy | Biomechanics research |

**Configuring Physics:**
You can adjust physics parameters in the **World** panel -> **Physics**:
- **Max Step Size**: Time per step (default 0.001s = 1000Hz).
- **Real Time Update Rate**: Target frequency (1000Hz).
- **Gravity**: Default (0, 0, -9.8).

## Troubleshooting Common Issues

### 1. Gazebo crashes on startup
**Cause:** Graphics driver issues or VM limitations.
**Fix:**
```bash
export SVGA_VGPU10=0  # For VMware
gazebo --verbose      # Check logs
```

### 2. "Real Time Factor" is low (< 0.5)
**Cause:** Simulation is too heavy for CPU.
**Fix:**
- Remove complex models.
- Increase `max_step_size` (reduces accuracy).
- Use a dedicated GPU.

### 3. Models look black or missing textures
**Cause:** Missing material scripts or lighting.
**Fix:**
- Ensure `<uri>model://sun</uri>` is in world file.
- Check `GAZEBO_MODEL_PATH` environment variable.

### 4. Robot falls through ground
**Cause:** Missing collision geometry or ground plane.
**Fix:**
- Check `<collision>` tags in URDF/SDF.
- Ensure ground plane is present at z=0.

## Best Practices

1. **Separate Visual and Collision Meshes**:
   - Visual: High-poly .dae/.stl (looks good).
   - Collision: Simple shapes (box/cylinder) or low-poly hull (fast physics).

2. **Use Model Database**:
   - Don't reinvent the wheel. Use standard models for tables, walls, etc.
   - `https://app.gazebosim.org/fuel`

3. **Keep It Simple**:
   - Start with simple primitives.
   - Only add complexity (mesh details) when necessary.

4. **Domain Randomization**:
   - Vary friction, mass, and lighting in simulation to make your AI robust.

## Summary

Gazebo is a powerful tool that simulates the physical world for your robot. By mastering:
- **World Building**: Creating environments.
- **Physics Configuration**: Tuning the engine.
- **ROS Integration**: Connecting to your code.

You create the "Digital Twin" where your Physical AI will learn and evolve.

---

**Next:** Learn how to create robot models and add sensors in [Robot Models and Sensors](./robot-models-sensors-gazebo.md).