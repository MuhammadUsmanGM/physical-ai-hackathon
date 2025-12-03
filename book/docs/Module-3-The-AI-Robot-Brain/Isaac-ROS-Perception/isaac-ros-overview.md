---
id: isaac-ros-overview
title: Isaac ROS and Perception
sidebar_label: Isaac ROS Overview
---

# Isaac ROS and Perception

**Isaac ROS** is a collection of hardware-accelerated ROS 2 packages (GEMs) that allow your robot to perform complex perception tasks (SLAM, Object Detection) in real-time on NVIDIA GPUs.

## Why Isaac ROS?

Standard ROS nodes run on the CPU. This is fine for simple tasks, but modern AI requires massive parallel processing.

| Task | CPU (Standard ROS) | GPU (Isaac ROS) |
|------|--------------------|-----------------|
| **Visual SLAM** | 10-15 FPS | 60+ FPS |
| **Object Detection** | 5 FPS (YOLO) | 30+ FPS (YOLO + TensorRT) |
| **3D Reconstruction** | Slow, Low Res | Real-time, High Res (Nvblox) |

## Architecture: NITROS and CUDA

Isaac ROS uses **NITROS** (NVIDIA Isaac Transport for ROS), which optimizes message passing.
- Instead of serializing/deserializing messages between nodes (slow), NITROS passes pointers to GPU memory (zero-copy).
- This means an image captured by the camera stays in GPU memory through the entire pipeline (Rectification -> Inference -> SLAM).

## Installation Guide (Docker)

Isaac ROS is best run inside a Docker container to manage dependencies (CUDA, TensorRT).

### Prerequisites
- **OS**: Ubuntu 20.04/22.04
- **Hardware**: NVIDIA Jetson Orin OR x86 PC with RTX GPU
- **Software**: Docker, NVIDIA Container Toolkit

### Step 1: Setup Workspace
```bash
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

### Step 2: Clone Packages
For this module, we need VSLAM and Nvblox.
```bash
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
```

### Step 3: Build and Run Container
```bash
cd ~/isaac_ros_ws/src/isaac_ros_common
./scripts/run_dev.sh
```
*This command builds the Docker image and drops you into a shell inside the container.*

### Step 4: Build ROS Packages
Inside the container:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Key Packages (GEMs)

### 1. Isaac ROS Visual SLAM
Provides robust localization in GPS-denied environments using stereo cameras.
- **Input**: Stereo Images + IMU
- **Output**: Robot Pose (Odometry)

### 2. Nvblox
Builds a 3D map of the environment in real-time for navigation.
- **Input**: Depth Image + Pose
- **Output**: 3D Costmap (ESDF) for Nav2

### 3. Isaac ROS DNN Inference
Runs deep learning models (YOLO, DOPE) using TensorRT.
- **Input**: Image
- **Output**: Bounding Boxes / Segmentation Masks

## Hands-On: Running Visual SLAM

Let's run VSLAM using a pre-recorded bag file (or simulation).

### 1. Download Data
```bash
# Inside container
sudo apt-get install -y ros-humble-isaac-ros-test
```

### 2. Launch VSLAM
```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
```

### 3. Visualize in RViz
Open RViz2 on your host machine (or inside container if GUI is enabled).
- **Fixed Frame**: `map`
- **Add Display**: `TF`, `Odometry` (Topic: `/visual_slam/tracking/odometry`)

You should see the robot's path being traced as it moves.

## Summary

Isaac ROS unlocks the "Supercomputer" inside your robot. By moving heavy computation to the GPU, you free up the CPU for high-level logic and decision making.

---

**Next:** Deploy this brain to the body in [Jetson Platform and Navigation](./jetson-platform-navigation.md).