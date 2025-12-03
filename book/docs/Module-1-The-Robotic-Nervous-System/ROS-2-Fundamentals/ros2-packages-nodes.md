---
id: ros2-packages-nodes
title: Creating and Managing ROS 2 Packages and Nodes  
sidebar_label: Packages and Nodes
---

# Creating and Managing ROS 2 Packages and Nodes

Packages are the fundamental unit of organization in ROS 2. This chapter teaches you how to create, structure, and manage ROS 2 packages, and how to write nodes in both Python and C++.

## Understanding ROS 2 Packages

**What is a Package?**
A package is a directory containing:
- Source code (C++, Python, or both)
- Build configuration (CMakeLists.txt, package.xml)
- Dependencies declaration  
- Launch files and configuration
- Documentation and tests

**Why Packages?**
- **Modularity**: Separate concerns into logical units
- **Reusability**: Share packages across projects
- **Dependencies**: Declare what your code needs
- **Build System**: Automated compilation and installation

## Package Types

ROS 2 supports two build systems:

| Build Type | Language | Build Tool | Use Case |
|------------|----------|------------|----------|
| `ament_python` | Python | setuptools | Pure Python packages |
| `ament_cmake` | C++ (or mixed) | CMake | C++ or mixed Python/C++ |

**Recommendation:**
- Pure Python → `ament_python`
- C++ or mixed → `ament_cmake`

## Creating a Python Package

### Step 1: Create Package

```bash
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_py_pkg \
  --dependencies rclpy std_msgs

Example output:
# going to create a new package
# package name: my_py_pkg  
# destination directory: /home/user/ros2_ws/src
# package format: 3
# version: 0.0.0
# ...
```

### Step 2: Package Structure

```
my_py_pkg/
├── my_py_pkg/           # Python module
│   └── __init__.py
├── resource/            # Package marker
│   └── my_py_pkg
├── test/                # Tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml          # Package metadata
├── setup.py             # Python setup config
├── setup.cfg            # Install config
└── README.md            # Documentation
```

### Step 3: Understanding package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_py_pkg</name>
  <version>0.0.0</version>
  <description>My Python ROS 2 package</description>
  <maintrainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Dependency Types:**
- `<buildtool_depend>`: Tools needed for building
- `<depend>`: Needed at build AND runtime
- `<build_depend>`: Only for building
- `<exec_depend>`: Only at runtime
- `<test_depend>`: Only for testing

### Step 4: Writing a Python Publisher Node

Create `my_py_pkg/my_py_pkg/publisher_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    """
    Simple publisher node that sends messages periodically.
    """
    def __init__(self):
        # Initialize node with name
        super().__init__('minimal_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(
            String,          # Message type
            'chatter',       # Topic name
            10               # QoS history depth
        )
        
        # Create timer (0.5 seconds)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.counter = 0

    def timer_callback(self):
        """Called every timer_period seconds."""
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        self.counter += 1

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)
    
    # Create node
    node = MinimalPublisher()
    
    # Spin (process callbacks)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Writing a Python Subscriber Node

Create `my_py_pkg/my_py_pkg/subscriber_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    """
    Simple subscriber node that receives and prints messages.
    """
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """Called when message received."""
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 6: Configure setup.py

Edit `setup.py` to register executables:

```python
from setuptools import setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Example Python package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = my_py_pkg.publisher_node:main',
            'subscriber = my_py_pkg.subscriber_node:main',
        ],
    },
)
```

**Entry Points Explained:**
```
'executable_name = package.module:function'
```

### Step 7: Build and Run

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select my_py_pkg

# Source workspace
source install/setup.bash

# Run publisher
ros2 run my_py_pkg publisher

# In another terminal (source first!)
ros2 run my_py_pkg subscriber
```

## Creating a C++ Package

### Step 1: Create Package

```bash
cd ~/ros2_ws/src

ros2 pkg create --build-type ament_cmake my_cpp_pkg \
  --dependencies rclcpp std_msgs
```

### Step 2: Package Structure

```
my_cpp_pkg/
├── include/
│   └── my_cpp_pkg/     # Header files
├── src/                # Source files
├── CMakeLists.txt      # Build configuration
├── package.xml         # Package metadata
└── README.md
```

### Step 3: Writing a C++ Publisher

Create `my_cpp_pkg/src/publisher_node.cpp`:

```cpp
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    // Create publisher
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    
    // Create timer (500ms)
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&MinimalPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS 2: " + std::to_string(count_++);
    
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

### Step 4: Configure CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Build publisher executable
add_executable(publisher src/publisher_node.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

# Install executables
install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### Step 5: Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_cpp_pkg
source install/setup.bash

ros2 run my_cpp_pkg publisher
```

## Advanced Node Features

### 1. Parameters in Nodes

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with defaults
        self.declare_parameter('my_string', 'default_value')
        self.declare_parameter('my_int', 42)
        self.declare_parameter('my_double', 3.14)
        
        # Get parameter values
        my_string = self.get_parameter('my_string').value
        my_int = self.get_parameter('my_int').value
        
        self.get_logger().info(f'String: {my_string}')
        self.get_logger().info(f'Int: {my_int}')
        
        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        """Called when parameters change."""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')
        return SetParametersResult(successful=True)
```

**Set parameter at runtime:**
```bash
ros2 param set /parameter_node my_int 100
```

### 2. Service Client and Server

**Server:**
```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

**Client:**
```python
class MinimalClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        future = self.client.call_async(request)
        return future
```

### 3. Launch Files

Create `my_py_pkg/launch/my_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_py_pkg',
            executable='publisher',
            name='my_publisher',
            output='screen',
            parameters=[{
                'my_param': 'value'
            }]
        ),
        Node(
            package='my_py_pkg',
            executable='subscriber',
            name='my_subscriber',
            output='screen'
        ),
    ])
```

**Install launch files in CMakeLists.txt:**
```cmake
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

**Run launch file:**
```bash
ros2 launch my_py_pkg my_launch.py
```

## Best Practices

### 1. Package Naming
- Lowercase, underscores for spaces
- Descriptive: `my_robot_navigation` not `nav`
- Consistent prefix: `myrobot_*`

### 2. Node Organization
```
src/
├── drivers/          # Hardware interfaces
├── perception/       # Sensor processing
├── planning/         # Decision making
└── control/          # Actuation
```

### 3. Dependencies
- Minimal: Only include what you use
- Specific: Declare exact dependencies
- Document: Explain why each dependency

### 4. Logging Levels
```python
self.get_logger().debug('Detailed info')
self.get_logger().info('General info')
self.get_logger().warn('Warning')
self.get_logger().error('Error')
self.get_logger().fatal('Critical error')
```

## Summary

You now know how to:
- ✓ Create Python and C++ packages
- ✓ Write publishers and subscribers
- ✓ Use parameters
- ✓ Create services
- ✓ Write launch files
- ✓ Follow best practices

## Key Takeaways

✓ Packages organize code into reusable modules  
✓ Python packages use `ament_python`, C++ uses `ament_cmake`  
✓ Nodes are executables that perform specific functions  
✓ Use package.xml to declare dependencies  
✓ Launch files start multiple nodes simultaneously  

## Practice Exercises

1. **Create Your Own Package:**
   - Create a package `my_robot_controller`
   - Write a publisher that sends robot commands
   - Write a subscriber that receives sensor data
   - Add parameters for configuration

2. **Service Implementation:**
   - Create a service to compute inverse kinematics
   - Test with `ros2 service call`

3. **Launch File:**
   - Create launch file for all nodes
   - Add parameters via launch file
   - Run and verify

---

**Module 1 Complete!** You've mastered ROS 2 fundamentals. Next: [Module 2: Digital Twin (Simulation)](../../Module-2-The-Digital-Twin/index.md)