---
id: 'module-1'
title: The Robotic Nervous System (ROS 2)
sidebar_label: Module Overview
---

# Module 1: The Robotic Nervous System (ROS 2)

## Focus: Middleware for robot control.

This module introduces ROS 2 (Robot Operating System 2), the middleware that serves as the nervous system for robotic applications. Students will learn to develop distributed robotic systems using ROS 2's communication patterns, package management, and development tools.

> **Analogy:** Just as the human nervous system coordinates messages between the brain and body, ROS 2 coordinates communication between different software components in a robot system.

## Why ROS 2 Matters for Physical AI

ROS 2 is the industry standard for robotics software development, used by:
- **Leading Companies**: Tesla (humanoid robots), NASA (Mars rovers), Boston Dynamics, Toyota Research
- **Research Institutions**: MIT, Stanford, CMU, ETH Zurich
- **Commercial Robots**: 80%+ of commercial mobile robots use ROS

**Key Advantages:**
- **Modularity**: Separate concerns (perception, planning, control) into independent nodes
- **Scalability**: Run on single devices or distributed across multiple computers
- **Real-time**: DDS middleware enables hard real-time performance
- **Community**: 3000+ packages, extensive libraries, active support

## Learning Objectives

By the end of this module, you will be able to:

### Conceptual Understanding
- **Explain** the ROS 2 architecture and how it differs from ROS 1
- **Describe** the publish-subscribe pattern and when to use topics vs services vs actions
- **Understand** Quality of Service (QoS) policies and their impact on communication
- **Identify** the role of DDS middleware in distributed systems

### Practical Skills
- **Install** and configure ROS 2 Humble on Ubuntu 22.04
- **Create** ROS 2 packages using both Python and C++
- **Implement** publishers, subscribers, services, and action servers/clients
- **Write** launch files to start complex multi-node systems
- **Debug** ROS 2 systems using CLI tools and rqt

### Advanced Application
- **Design** multi-node robotic systems with appropriate communication patterns
- **Optimize** node communication with QoS policies
- **Integrate** sensors and actuators into ROS 2 control loops
- **Build** a complete robot control system from scratch

## Module Structure

This 3-week module (Weeks 3-5) is organized into three progressive topics:

### Week 3: ROS 2 Architecture and Core Concepts
**Learning Focus:** Understanding the foundational concepts

**Topics:**
1. **ROS 2 Architecture**
   - DDS middleware layer
   - Node-based architecture
   - Communication patterns overview
   - Comparison with ROS 1

2. **Core Communication Patterns**
   - Topics (publish-subscribe)
   - Services (request-response)
   - Actions (goal-oriented)
   - Parameters (configuration)

**Deliverables:**
- Quiz on ROS 2 concepts
- Communication pattern selection exercise

### Week 4: Development Environment and Package Management
**Learning Focus:** Setting up and working with ROS 2

**Topics:**
1. **Environment Setup**
   - ROS 2 Humble installation
   - Workspace configuration
   - Build system (colcon)
   - IDE setup (VS Code + extensions)

2. **Package Creation and Management**
   - ament_cmake vs ament_python
   - package.xml structure
   - Dependencies and versioning
   - Best practices

**Deliverables:**
- Configured development environment
- Created custom package

### Week 5: Building ROS 2 Applications
**Learning Focus:** Writing nodes and integrating systems

**Topics:**
1. **Node Implementation**
   - Python rclpy examples
   - C++ rclcpp examples
   - Lifecycle nodes
   - Multi-threaded executors

2. **Launch Files and Parameters**
   - Python launch files
   - XML and YAML configurations
   - Parameter management
   - Namespaces and remapping

**Deliverables:**
- Multi-node application
- Launch file configuration
- **Module Project**: Simple robot controller

## ROS 2 Ecosystem Overview

Understanding the complete ROS 2 ecosystem:

```
┌─────────────────────────────────────────────┐
│         ROS 2 Application Layer             │
│  (Your nodes: Python/C++ code)              │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│         ROS 2 Client Libraries              │
│  rclpy (Python) | rclcpp (C++)              │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│            ROS 2 Core (rcl)                 │
│  Node management, graph API, time           │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│         DDS Middleware Layer                │
│  Fast DDS | Cyclone DDS | RTI Connext       │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│         Operating System (Linux)            │
│  Ubuntu 22.04 (recommended)                 │
└─────────────────────────────────────────────┘
```

## Prerequisites

### Required Knowledge

**Programming:**
- ✓ **Python 3.8+**: Object-oriented programming, decorators, async/await
- ✓ **Linux CLI**: bash commands, file permissions, environment variables
- ✓ **Git**: Clone, commit, branch, merge basics

**Optional but Helpful:**
- C++ basics (classes, templates, smart pointers)
- CMake build system
- Networking concepts (TCP/IP, UDP)
- XML and YAML formats

### Required Software

**Operating System:**
- Ubuntu 22.04 LTS (strongly recommended)
- Alternative: Docker container with ROS 2
- Windows WSL2 (limited support)

**Hardware:**
- 4+ core CPU
- 8GB RAM minimum (16GB recommended)
- 20GB free disk space

## Assessment and Projects

### Weekly Assessments (40%)

| Week | Assessment | Weight | Type |
|------|------------|--------|------|
| **Week 3** | ROS 2 Concepts Quiz | 10% | Multiple choice + short answer |
| **Week 4** | Package Creation Lab | 15% | Hands-on coding |
| **Week 5** | Multi-node Application | 15% | Project implementation |

### Module Project: Robot Controller (40%)

Build a complete robot control system with:
- **Sensor node**: Publish simulated sensor data (camera, LiDAR, IMU)
- **Processing node**: Subscribe to sensors, compute outputs
- **Actuator node**: Receive commands, control motors (simulated)
- **Launch file**: Start all nodes with proper configuration
- **Documentation**: README with architecture diagram

**Evaluation Criteria:**
- Correct implementation of communication patterns (30%)
- Code quality and organization (25%)
- Launch file and configuration (20%)
- Documentation and architecture (25%)

### Participation and Labs (20%)

- Attendance and engagement
- Completion of in-class exercises
- Peer code reviews

## Key Tools and Commands

You'll master these essential ROS 2 tools:

**Node Management:**
```bash
ros2 run <package> <executable>    # Run a node
ros2 node list                     # List active nodes
ros2 node info <node_name>         # Node details
```

**Topic Operations:**
```bash
ros2 topic list                    # List all topics
ros2 topic echo <topic>            # Monitor messages
ros2 topic pub <topic> <type> <data>  # Publish manually
ros2 topic hz <topic>              # Check frequency
```

**Service and Action Commands:**
```bash
ros2 service list                  # List services
ros2 service call <service> <type> <request>
ros2 action list                   # List actions
ros2 action send_goal <action> <type> <goal>
```

**Build and Development:**
```bash
colcon build                       # Build workspace
colcon test                        # Run tests
source install/setup.bash          # Source workspace
```

## Real-World Connection

Throughout this module, you'll see how ROS 2 concepts apply to actual robots:

- **Autonomous Vehicles**: Topic-based sensor fusion, action-based navigation
- **Manipulation**: Service-based grasp planning, action-based motion execution
- **Multi-Robot Systems**: Namespaces for robot fleets, distributed discovery
- **Industrial Automation**: Real-time control with QoS guarantees

## Learning Resources

### Official Documentation
- ROS 2 Documentation: [https://docs.ros.org/](https://docs.ros.org/)
- ROS 2 Humble Tutorials: Design, CLI tools, client libraries
- DDS Foundation: Understanding the middleware

### Recommended Books
- **"A Gentle Introduction to ROS"** by Jason O'Kane (concepts apply to ROS 2)
- **"ROS Robotics Projects"** - Practical applications
- **"Programming Robots with ROS"** - O'Reilly Media

### Video Courses
- The Construct: ROS 2 for Beginners
- Udemy: ROS 2 Complete Course
- YouTube: Articulated Robotics ROS 2 series

### Community Resources
- ROS Discourse: [https://discourse.ros.org/](https://discourse.ros.org/)
- ROS Answers: Q&A platform
- GitHub: ros2/examples repository

## Getting Started

### Week 3 Preparation Checklist
- [ ] Install Ubuntu 22.04 (or prepare Docker environment)
- [ ] Complete ROS 2 installation (instructions in Environment Setup chapter)
- [ ] Verify installation with demo nodes
- [ ] Join ROS Discourse forum
- [ ] Clone course example repository

### Study Tips
1. **Practice Immediately**: Run examples after reading each section
2. **Modify Code**: Change parameters, experiment with different values
3. **Draw Diagrams**: Sketch node graphs for better understanding
4. **Use Visualization**: rqt_graph is invaluable for debugging
5. **Read Error Messages**: ROS 2 errors are usually descriptive

## Module Roadmap

```
Week 3: Foundations
  ↓ Understand architecture and patterns
Week 4: Tools and Environment
  ↓ Set up and create packages
Week 5: Implementation
  ↓ Build complete applications
Module Project: Integration
  ↓ Demonstrate mastery
Ready for Module 2: Simulation!
```

## Summary

ROS 2 is the foundation upon which modern robotic systems are built. Mastering ROS 2:
- **Enables** you to build complex, distributed robot software
- **Connects** you to the global robotics community
- **Prepares** you for industry roles (90% of robotics jobs require ROS knowledge)
- **Provides** tools to bring your robotic ideas to life

As you progress, remember: ROS 2 is a tool, not the goal. The goal is to build intelligent, capable Physical AI systems. ROS 2 is the communication infrastructure that makes that possible.

---

**Next:** Begin with [ROS 2 Architecture and Core Concepts](./ROS-2-Fundamentals/ros2-architecture.md) to understand the foundational architecture.