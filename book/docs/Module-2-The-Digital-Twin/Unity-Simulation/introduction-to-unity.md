---
id: introduction-to-unity
title: Introduction to Unity for Robotics
sidebar_label: Introduction to Unity
---

# Introduction to Unity for Robotics

Unity is a professional game engine that has become a powerhouse for robotics simulation, particularly for tasks requiring high-fidelity visuals (Computer Vision) and complex physical interactions (Manipulation).

## Unity vs. Gazebo: When to Use Which?

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Primary Focus** | Physics & Control | Visuals & Interaction |
| **Rendering** | Basic (OGRE) | Photorealistic (HDRP/URP) |
| **Physics** | Accurate (ODE/Dart) | Fast (PhysX/Havok) |
| **ROS Integration** | Native | via ROS-TCP-Connector |
| **Best For** | Navigation, Dynamics | Vision, Human-Robot Interaction |

**Use Unity when:**
- You need to train Computer Vision models (Synthetic Data).
- You are simulating Human-Robot Interaction.
- You need VR/AR interfaces.

## Setting Up Unity for Robotics

### Step 1: Install Unity Hub & Editor
1. Download **Unity Hub**.
2. Install **Unity 2022.3 LTS** (Long Term Support).
   - Include **Linux Build Support (Mono)** if deploying to Linux.

### Step 2: Create a Project
1. Open Unity Hub -> New Project.
2. Select **3D (URP)** or **3D (HDRP)** template.
   - **URP (Universal Render Pipeline)**: Faster, good for standard hardware.
   - **HDRP (High Definition Render Pipeline)**: Best visuals, requires strong GPU.

### Step 3: Install Unity Robotics Hub
The [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) contains essential tools.

1. Open **Window -> Package Manager**.
2. Click **+ -> Add package from git URL**.
3. Add: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Add: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

## Connecting Unity to ROS 2

Unity communicates with ROS 2 over TCP/IP using the **ROS-TCP-Endpoint**.

### 1. ROS 2 Side Setup
Install the endpoint package in your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
colcon build
source install/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### 2. Unity Side Setup
1. In Unity, go to **Robotics -> ROS Settings**.
2. Set **ROS IP Address** to your ROS 2 machine IP (or `127.0.0.1` if local).
3. Set **ROS Port** to `10000`.

## Importing Robots (URDF Importer)

Unity can import your URDF files directly.

1. **Assets -> Import Robot from URDF**.
2. Select your `.urdf` file.
3. Configure import settings:
   - **Axis Type**: Y-Axis Up (Unity standard) vs Z-Axis Up (ROS standard).
   - **Mesh Orientation**: Fix mesh rotation issues.
4. Click **Import**.

**Result**: A GameObject hierarchy with ArticulationBodies (Unity's physics components for robots).

## Simulating Sensors in Unity

### Camera (RGB)
1. Add a **Camera** component to your robot.
2. Add the **Camera Publisher** script (from Robotics Hub).
3. Set the topic name (e.g., `/camera/image_raw`).
4. Set the frame rate (e.g., 30 Hz).

### LiDAR (Raycast)
Unity simulates LiDAR using physics raycasts.
1. Create a script `LidarSensor.cs`.
2. Use `Physics.Raycast` in a loop.
3. Publish `sensor_msgs/LaserScan`.

```csharp
// Simple Raycast Lidar Snippet
void Update() {
    float[] ranges = new float[360];
    for (int i = 0; i < 360; i++) {
        if (Physics.Raycast(transform.position, rotation * Vector3.forward, out hit)) {
            ranges[i] = hit.distance;
        } else {
            ranges[i] = maxRange;
        }
    }
    // Publish ranges to ROS...
}
```

## Synthetic Data Generation

One of Unity's superpowers is generating labeled data for AI training.

**Perception SDK**:
Unity provides a Perception SDK to automatically label objects.

1. **Install Perception Package** via Package Manager.
2. Add **Labeling Component** to target objects (e.g., "Cup", "Person").
3. Add **Perception Camera** to your scene.
4. Configure **Randomizers**:
   - **Lighting**: Change color, intensity, position.
   - **Texture**: Swap materials on objects/background.
   - **Pose**: Randomize object positions.

**Workflow:**
1. Run Simulation.
2. Unity renders thousands of frames with varying conditions.
3. JSON/XML labels are generated automatically (Bounding Boxes, Segmentation Masks).
4. Train your YOLO/Mask-RCNN model on this data.

## Sim-to-Real: Domain Randomization

To ensure your AI works in the real world, you must bridge the "Reality Gap".

**Techniques:**
1. **Visual Randomization**: Randomize textures, lights, and camera noise.
2. **Physics Randomization**: Randomize friction, mass, and joint damping.
3. **Dynamics Randomization**: Add random forces/torques to the robot.

```csharp
// Example: Randomizing Friction
void RandomizePhysics() {
    float friction = Random.Range(0.5f, 1.0f);
    collider.material.dynamicFriction = friction;
    collider.material.staticFriction = friction;
}
```

## Summary

Unity transforms simulation from a physics testbed into a **data factory**. By combining:
- **ROS-TCP-Connector** for control.
- **URDF Importer** for modeling.
- **Perception SDK** for data generation.

You can train sophisticated Vision-Language-Action models that are robust enough for the real world.

---

**Module 2 Complete!** You now have the tools to build the world. Next, we give the robot a brain in [Module 3: The AI-Robot Brain (NVIDIA Isaac)](../../Module-3-The-AI-Robot-Brain/index.md).