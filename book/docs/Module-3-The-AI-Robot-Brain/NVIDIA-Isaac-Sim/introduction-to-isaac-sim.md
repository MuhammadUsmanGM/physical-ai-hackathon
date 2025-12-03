---
id: introduction-to-isaac-sim
title: Introduction to NVIDIA Isaac Sim
sidebar_label: Introduction to Isaac Sim
---

# Introduction to NVIDIA Isaac Sim

**NVIDIA Isaac Sim™** is a reference application built on **NVIDIA Omniverse™** for designing, simulating, testing, and training AI-based robots. Unlike Gazebo, which focuses on physics, Isaac Sim focuses on **photorealism** and **physics fidelity** to close the Sim-to-Real gap.

## Why Isaac Sim?

| Feature | Gazebo | Isaac Sim |
|---------|--------|-----------|
| **Engine** | OGRE (Game-like) | RTX (Ray Tracing) |
| **Physics** | ODE/Bullet | PhysX 5.0 (GPU Accelerated) |
| **Assets** | URDF/SDF | USD (Universal Scene Description) |
| **Scripting** | C++/Python | Python (Async) |
| **Hardware** | CPU Heavy | GPU Heavy (RTX Required) |

## Installation Guide

### Prerequisites
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11
- **GPU**: NVIDIA RTX 2070 or higher (8GB+ VRAM)
- **Driver**: NVIDIA Driver 525.85+

### Step 1: Install Omniverse Launcher
1. Download the **Omniverse Launcher** from the [NVIDIA website](https://www.nvidia.com/en-us/omniverse/download/).
2. Install and log in with your NVIDIA Developer account.

### Step 2: Install Isaac Sim
1. Open the **Exchange** tab in the Launcher.
2. Search for "Isaac Sim".
3. Click **Install**.
4. Once installed, go to **Library** and click **Launch**.

### Step 3: Install Nucleus Service
Nucleus is the database server for sharing assets.
1. In the Launcher, go to the **Nucleus** tab.
2. Click **Add Local Nucleus Service**.
3. Create an admin account.

## The Omniverse Interface

When you launch Isaac Sim, you'll see the **Viewport** and several panels:

1. **Viewport (Center)**: The 3D view.
   - **Right Click + WASD**: Fly around.
   - **Alt + Left Click**: Orbit.
   - **Alt + Middle Click**: Pan.

2. **Stage (Right)**: The hierarchy of objects (like the World panel in Gazebo).
   - Everything is a "Prim" (Primitive).

3. **Property (Right-Bottom)**: Settings for the selected Prim.
   - Transform (Position, Rotation).
   - Physics (Mass, Collision).
   - Material (Textures).

4. **Content (Bottom)**: File browser for your Nucleus server.
   - Access NVIDIA Assets: `omniverse://localhost/NVIDIA/Assets/Isaac`.

## Understanding USD (Universal Scene Description)

Isaac Sim uses **USD**, a file format developed by Pixar.

- **Layering**: You can have a "Robot" layer and an "Environment" layer. Changes in one don't destroy the other.
- **Non-Destructive**: You can "override" properties (e.g., make the robot red) without changing the original file.
- **Composition**: A scene is composed of many references to other USD files.

## Your First Simulation: "Hello World"

Let's spawn a cube and make it fall.

### Method 1: GUI
1. **Create -> Shape -> Cube**.
2. Select the Cube in the Stage.
3. In **Property** panel, scroll to **Physics**.
4. Click **Add -> Physics -> Rigid Body with Colliders Preset**.
5. Press **Play** (Spacebar). The cube falls!

### Method 2: Python Scripting
Isaac Sim has a built-in Script Editor (**Window -> General -> Script Editor**).

```python
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.world import World
import numpy as np

# Initialize the world
world = World()
world.scene.clear()

# Add a ground plane
world.scene.add_default_ground_plane()

# Add a dynamic cube (Red)
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/MyCube",
        name="my_cube",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([1.0, 0, 0]),
    )
)

# Reset the world to apply changes
world.reset()

# Run the simulation loop
for i in range(500):
    world.step(render=True)
```

## Importing a Robot (URDF to USD)

Most robots exist as URDFs. Isaac Sim has a powerful importer.

1. **Isaac Utils -> Workflows -> URDF Importer**.
2. **Input File**: Select your `.urdf` file.
3. **Import Settings**:
   - **Fix Base Link**: Check this if it's a manipulator arm. Uncheck for mobile robots.
   - **Joint Drive Type**: Select "Position" for arms, "Velocity" for wheels.
4. Click **Import**.

The robot is converted to USD and appears in the stage.

## Summary

Isaac Sim is a leap forward in simulation technology. It requires a mindset shift from "files and meshes" (Gazebo) to "stages and layers" (USD).

**Key Takeaways:**
- **Nucleus** stores your files.
- **USD** is the file format.
- **Python** is the scripting language.

---

**Next:** Learn how to rig robots and create complex scenes in [Robot Assets and Simulation](./robot-assets-simulation.md).