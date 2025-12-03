---
id: ros2-packages-nodes
title: Creating and Managing ROS 2 Packages and Nodes
sidebar_label: Packages and Nodes
---

# Creating and Managing ROS 2 Packages and Nodes

## Package Structure and Creation

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

## Writing Your First ROS 2 Node

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

## Build System Configuration

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