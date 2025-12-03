---
id: robot-assets-simulation
title: Robot Assets and Simulation in Isaac Sim
sidebar_label: Robot Assets and Simulation
---

# Robot Assets and Simulation in Isaac Sim

In this chapter, we dive deeper into creating "Digital Twins." You will learn how to rig robots, control them with Python, and generate synthetic data.

## The USD Workflow: Composition & Layering

USD (Universal Scene Description) is powerful because it allows **composition**.

### 1. References
Instead of copying a robot into your scene, you **reference** it.
- If you update the original robot file, all scenes referencing it update automatically.

### 2. Layers
A stage can have multiple layers.
- **Root Layer**: The main file.
- **Sub-Layer**: A file included in the root.
- **Delta Layer**: Stores *changes* (e.g., "Robot is at x=5") without modifying the original robot file.

## Rigging a Robot (Articulation)

If you import a URDF, it's already rigged. If you bring in a raw CAD model (OBJ/STL), you must rig it.

### Step 1: Articulation Root
1. Select the root link of your robot (e.g., `base_link`).
2. **Property Panel -> Add -> Physics -> Articulation Root**.
3. This tells PhysX "this is a robot with joints."

### Step 2: Joint Drives
Joints in Isaac Sim are controlled by **Drives**.
1. Select a Joint (e.g., `wheel_joint`).
2. **Property Panel -> Drive**.
3. **Stiffness (P-Gain)**: High for position control (Arms). Low for velocity control (Wheels).
4. **Damping (D-Gain)**: Resistance to motion.

## Visual Scripting: Action Graph

You can create logic without writing code using **Action Graph** (similar to Blueprints in Unreal).

**Example: Keyboard Control**
1. **Window -> Visual Scripting -> Action Graph**.
2. Create a new Graph.
3. Add Nodes:
   - **On Playback Tick**: Runs every frame.
   - **Read Keyboard Input**: Detects 'W' key.
   - **Articulation Controller**: Applies velocity to wheels.
4. Connect them. Now you can drive your robot with WASD!

## Python API: Controlling Robots

For serious AI work, we use the Python API.

### The Core API (`omni.isaac.core`)
This provides a high-level abstraction over USD.

```python
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

# Wrap the USD prim as a Robot object
my_robot = Robot(prim_path="/World/Franka", name="franka")

# Initialize
my_robot.initialize()

# Send a command (Position Control)
# Move joints to specific angles
action = ArticulationAction(joint_positions=np.array([0.5, -0.5, 0.0, -1.5, 0.0, 1.5, 0.7]))
my_robot.apply_action(action)
```

### The World Class
Manages the simulation loop and physics.

```python
from omni.isaac.core import World

world = World(stage_units_in_meters=1.0)
world.scene.add(my_robot)
world.reset()

while simulation_app.is_running():
    world.step(render=True)
```

## Synthetic Data Generation (Replicator)

**Omniverse Replicator** is the engine for generating training data.

### 1. Randomization
We want to vary the scene to make the AI robust.

```python
import omni.replicator.core as rep

with rep.new_layer():
    # Randomize Light
    light = rep.create.light(intensity=1000, light_type="sphere")
    with rep.trigger.on_frame(interval=10):
        with light:
            rep.modify.pose(position=rep.distribution.uniform((-5, -5, 5), (5, 5, 10)))
            rep.modify.attribute("color", rep.distribution.uniform((0, 0, 0), (1, 1, 1)))

    # Randomize Texture
    cube = rep.create.cube()
    with rep.trigger.on_frame(interval=10):
        with cube:
            rep.randomizer.texture(textures=["/Textures/Wood.png", "/Textures/Metal.png"])
```

### 2. Annotation (Ground Truth)
Replicator automatically generates labels.

```python
# Create a writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="~/isaac_data", rgb=True, bounding_box_2d_tight=True)

# Attach to render product (Camera)
render_product = rep.create.render_product("/World/Camera", (1024, 1024))
writer.attach(render_product)

# Run generation
rep.orchestrator.run()
```

**Output**: A folder full of images and JSON files with bounding boxes, ready for YOLO training.

## Summary

In this chapter, you learned:
- **USD Composition**: How to assemble scenes efficiently.
- **Rigging**: Making static meshes move.
- **Action Graph**: Visual coding for quick tests.
- **Replicator**: Generating infinite training data.

---

**Next:** Move to the edge with [Isaac ROS Overview](../Isaac-ROS-Perception/isaac-ros-overview.md).