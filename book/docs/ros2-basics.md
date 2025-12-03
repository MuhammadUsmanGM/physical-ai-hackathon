---
id: ros2-basics
title: ROS 2 Basics
---

# ROS 2 Basics

## Introduction to ROS 2

The Robot Operating System 2 (ROS 2) serves as the nervous system for robotic applications, providing the infrastructure and tools necessary for developing complex robotic systems. Unlike traditional operating systems, ROS 2 is middleware that enables communication between different software components running on potentially distributed systems.

### What is ROS 2?

ROS 2 is an open-source framework for writing robot software. It provides libraries and tools to help software developers create robot applications. ROS 2 addresses the limitations of ROS 1, particularly in areas of security, real-time capabilities, and multi-robot systems.

**Core Philosophy**: ROS 2 follows a distributed computing model where computation is divided among separate processes called nodes. These nodes communicate with each other using a publish-subscribe messaging pattern, services, and actions.

**Middleware Foundation**: ROS 2 is built on DDS (Data Distribution Service), which provides a standardized publisher-subscriber infrastructure and handles the complexities of message passing, discovery, and data serialization.

### Key Differences Between ROS 1 and ROS 2

| Aspect | ROS 1 | ROS 2 |
|--------|--------|--------|
| **Architecture** | Master-based centralized | DDS-based distributed |
| **Multi-robot Support** | Complex setup required | Native support |
| **Security** | No built-in security | Comprehensive security framework |
| **Real-time Support** | Limited | Improved real-time capabilities |
| **Cross-platform** | Primarily Linux | Linux, macOS, Windows |
| **Quality of Service** | Basic | Advanced QoS policies |

### ROS 2 Architecture and Core Concepts

**Nodes**: The fundamental building blocks of ROS 2 applications. Each node performs specific functions and communicates with other nodes through messages, services, or actions.

**Topics**: Named buses over which nodes exchange messages. Topics implement a publisher-subscriber communication pattern where one or more nodes publish data to a topic, and one or more nodes subscribe to that topic to receive the data.

**Services**: Request-response communication pattern where a client sends a request and receives a response from a server. Useful for synchronous operations.

**Actions**: More complex communication pattern for long-running tasks that require feedback, goal preemption, and result reporting. Actions are crucial for robot navigation and manipulation.

## Setting Up Your ROS 2 Development Environment

### Installing ROS 2

ROS 2 supports multiple distributions, with the latest being Humble Hawksbill (LTS) and Jazzy Jalisco. For this course, we'll use ROS 2 Humble as it provides long-term support and extensive robot compatibility.

**Prerequisites**:
- Ubuntu 22.04 LTS (recommended) or other supported platforms
- Python 3.8 or higher
- At least 8GB RAM (16GB recommended)
- Multi-core processor for optimal performance

### Creating Your First ROS 2 Workspace

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

### Essential ROS 2 Commands

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

## Creating and Managing ROS 2 Packages

### Package Structure and Creation

A ROS 2 package typically follows this structure:

```
my_robot_package/
├── CMakeLists.txt         # Build instructions for C++
├── package.xml           # Package metadata
├── src/                  # C++ source files
├── include/              # C++ header files
├── scripts/              # Python scripts (executable)
├── launch/               # Launch files
├── config/               # Configuration files
├── test/                 # Unit tests
└── README.md             # Documentation
```

**Creating a Package**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclpy std_msgs my_robot_package
```

### Writing Your First ROS 2 Node

**Python Node Example**:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**C++ Node Example**:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclpy::init(argc, argv);
    rclpy::spin(std::make_shared<MinimalPublisher>());
    rclpy::shutdown();
    return 0;
}
```

### Build System Configuration

For CMake packages, your `CMakeLists.txt` should include:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## Publisher-Subscriber Communication Pattern

### Understanding Topics and Messages

Topics enable asynchronous, many-to-many communication between nodes. Publishers send data to topics, and subscribers receive data from topics. This decoupling allows nodes to be developed and tested independently.

**Message Types**: ROS 2 provides standard message types in packages like `std_msgs` (basic types), `geometry_msgs` (geometric types), and `sensor_msgs` (sensor data). You can also define custom message types.

**Creating Custom Messages**:
1. Create a `msg` directory in your package
2. Define your message in a `.msg` file:
```
# MyCustomMessage.msg
string name
int32 id
float64[] position
bool is_active
```
3. Update your `package.xml` to include the message generation dependency
4. Modify your `CMakeLists.txt` to build the custom messages

### Quality of Service (QoS) Settings

QoS policies control the behavior of topic communication, including reliability, durability, and history settings. These are crucial for real-time and safety-critical robotics applications.

**Reliability Policy**:
- `RMW_QOS_POLICY_RELIABILITY_RELIABLE`: All messages are delivered (with retries)
- `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`: Messages may be lost but with lower latency

**Durability Policy**:
- `RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL`: Late-joining subscribers receive last known value
- `RMW_QOS_POLICY_DURABILITY_VOLATILE`: No historical data is retained

## Services and Actions

### Services - Request/Response Communication

Services are synchronous communication methods where a client sends a request to a service server and waits for a response. This is ideal for operations that need to return a result immediately.

**Service Implementation**:
1. Define a service in an `.srv` file (e.g., `AddTwoInts.srv`):
```
int64 a
int64 b
---
int64 sum
```

2. Implement service server:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Actions - Long-Running Tasks

Actions extend services to handle long-running operations that require feedback, goal preemption, and result reporting. They're essential for tasks like robot navigation and manipulation.

**Action Structure**:
- Goal: Request sent to initiate the action
- Feedback: Periodic updates on progress
- Result: Final outcome of the action

**Action Implementation**:
```python
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Launch Files for Complex Systems

Launch files allow you to start multiple nodes with specific configurations simultaneously. This is essential for real-world robotic systems that consist of many interconnected components.

### Creating Launch Files

Launch files can be written in Python or XML. For this course, we'll focus on Python launch files due to their flexibility and power.

**Basic Launch File**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='my_talker',
            parameters=[
                {'param_name': 'param_value'}
            ],
            remappings=[
                ('original_topic', 'new_topic')
            ]
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='my_listener'
        )
    ])
```

### Advanced Launch Features

- **Arguments**: Pass parameters to launch files from the command line
- **Conditions**: Start nodes based on conditions
- **Composable Nodes**: Run multiple nodes in a single process for efficiency
- **Lifecycle Nodes**: Manage node state transitions (configure, activate, etc.)

## Debugging and Monitoring Tools

### ROS 2 Command-Line Tools

- `ros2 doctor`: Diagnose common ROS 2 problems
- `ros2 bag record`: Record data from topics for later analysis
- `ros2 bag play`: Replay recorded data
- `rqt_graph`: Visualize the node graph
- `rqt_plot`: Plot numeric values from topics

### Interactive Debugging

- **RViz2**: 3D visualization tool for robot data
- **rqt**: Extensible GUI framework with various plugins
- **Built-in Logging**: ROS 2 provides structured logging capabilities

### Performance Monitoring

- **Resource Utilization**: Monitor CPU, memory, and bandwidth usage
- **Message Rates**: Ensure topics are publishing at expected frequencies
- **Timing Analysis**: Verify real-time constraints are met

## ROS 2 in the Context of the Course

ROS 2 serves as the foundational middleware for all modules in this course. Understanding ROS 2 is crucial because:

1. **Module Integration**: All modules (simulation, perception, control) communicate via ROS 2
2. **Hardware Abstraction**: ROS 2 provides consistent interfaces to different hardware platforms
3. **Community Ecosystem**: Leverage existing ROS 2 packages and tools from the robotics community
4. **Real-World Relevance**: Most professional robotics applications use ROS 2

The concepts learned in this module will directly apply to subsequent modules where you'll connect ROS 2 nodes to simulation environments, integrate perception systems, and control humanoid robots. Mastering these fundamentals will enable you to build complex, distributed robotic systems that embody the principles of Physical AI.