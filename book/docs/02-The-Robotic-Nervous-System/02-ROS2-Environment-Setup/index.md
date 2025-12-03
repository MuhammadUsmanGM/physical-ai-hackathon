---
id: ros2-environment-setup
title: Setting Up Your ROS 2 Development Environment
slug: /module-02/ros2-environment-setup
---

# Setting Up Your ROS 2 Development Environment

## Installing ROS 2

ROS 2 supports multiple distributions, with the latest being Humble Hawksbill (LTS) and Jazzy Jalisco. For this course, we'll use ROS 2 Humble as it provides long-term support and extensive robot compatibility.

**Prerequisites**:
- Ubuntu 22.04 LTS (recommended) or other supported platforms
- Python 3.8 or higher
- At least 8GB RAM (16GB recommended)
- Multi-core processor for optimal performance

## Creating Your First ROS 2 Workspace

A workspace is a directory where you modify, build, and install ROS 2 packages. The standard structure includes:

```bash
ros2_ws/
├── src/
│   ├── my_robot_driver/
│   ├── my_robot_control/
│   └── my_robot_description/
├── build/
├── install/
└── log/
```

**Step-by-step Workspace Setup**:

1. Create the workspace directory:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Source the ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
```

3. Build the workspace:
```bash
colcon build
```

4. Source the workspace setup file:
```bash
source install/setup.bash
```

## Essential ROS 2 Commands

**Node Management**:
- `ros2 node list`: List all active nodes
- `ros2 node info <node_name>`: Get detailed information about a specific node
- `ros2 run <package_name> <executable_name>`: Run a node directly

**Topic Management**:
- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Subscribe to a topic and print messages
- `ros2 topic info <topic_name>`: Get information about a topic
- `ros2 topic pub <topic_name> <msg_type> <args>`: Publish a message to a topic

**Service Management**:
- `ros2 service list`: List all available services
- `ros2 service call <service_name> <service_type> <request_args>`: Call a service

**Parameter Management**:
- `ros2 param list`: List all parameters for a node
- `ros2 param get <node_name> <param_name>`: Get a parameter value
- `ros2 param set <node_name> <param_name> <value>`: Set a parameter value

This environment setup is crucial for developing and testing ROS 2-based robotic applications.