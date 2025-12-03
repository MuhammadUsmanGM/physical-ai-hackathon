---
id: ros2-architecture
title: ROS 2 Architecture and Core Concepts
sidebar_label: Architecture and Core Concepts
---

# ROS 2 Architecture and Core Concepts

The Robot Operating System 2 (ROS 2) serves as the nervous system for robotic applications, providing the infrastructure and tools necessary for developing complex robotic systems. Unlike traditional operating systems, ROS 2 is middleware that enables communication between different software components running on potentially distributed systems.

## What is ROS 2?

ROS 2 is an open-source framework for writing robot software. It provides libraries and tools to help software developers create robot applications. ROS 2 addresses the limitations of ROS 1, particularly in areas of security, real-time capabilities, and multi-robot systems.

**Core Philosophy**: ROS 2 follows a distributed computing model where computation is divided among separate processes called nodes. These nodes communicate with each other using a publish-subscribe messaging pattern, services, and actions.

**Middleware Foundation**: ROS 2 is built on DDS (Data Distribution Service), which provides a standardized publisher-subscriber infrastructure and handles the complexities of message passing, discovery, and data serialization.

## Key Differences Between ROS 1 and ROS 2

| Aspect | ROS 1 | ROS 2 |
|--------|--------|--------|
| **Architecture** | Master-based centralized | DDS-based distributed |
| **Multi-robot Support** | Complex setup required | Native support |
| **Security** | No built-in security | Comprehensive security framework |
| **Real-time Support** | Limited | Improved real-time capabilities |
| **Cross-platform** | Primarily Linux | Linux, macOS, Windows |
| **Quality of Service** | Basic | Advanced QoS policies |

## ROS 2 Architecture and Core Concepts

**Nodes**: The fundamental building blocks of ROS 2 applications. Each node performs specific functions and communicates with other nodes through messages, services, or actions.

**Topics**: Named buses over which nodes exchange messages. Topics implement a publisher-subscriber communication pattern where one or more nodes publish data to a topic, and one or more nodes subscribe to that topic to receive the data.

**Services**: Request-response communication pattern where a client sends a request and receives a response from a server. Useful for synchronous operations.

**Actions**: More complex communication pattern for long-running tasks that require feedback, goal preemption, and result reporting. Actions are crucial for robot navigation and manipulation.