---
id: building-ros2-packages
title: Building ROS 2 Packages with Python
slug: /module-02/building-ros2-packages
---

# Building ROS 2 Packages with Python

## Introduction

A **ROS 2 package** is the fundamental unit of organization for ROS code. Packages contain nodes, libraries, configuration files, and other resources needed for your robot application. In this chapter, you'll learn to create, build, and organize ROS 2 packages using Python, the primary language for rapid robotics development.

## Package Structure Overview

```
my_robot_package/
├── package.xml          # Package metadata and dependencies
├── setup.py             # Python package configuration
├── setup.cfg            # Additional Python setup configuration
├── resource/            # Package marker file
│   └── my_robot_package
├── my_robot_package/    # Python source code directory
│   ├── __init__.py
│   ├── my_node.py
│   └── utils.py
├── launch/              # Launch files
│   └── my_launch.py
├── config/              # Configuration files (YAML, etc.)
│   └── params.yaml
├── urdf/                # Robot descriptions
│   └── robot.urdf
└── test/                # Unit tests
    └── test_my_node.py
```

---

## Creating Your First ROS 2 Package

### Step 1: Navigate to Your Workspace

```bash
cd ~/ros2_ws/src
```

### Step 2: Create the Package

```bash
# Create a Python package with dependencies
ros2 pkg create --build-type ament_python my_robot_package \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs

# Explanation:
# --build-type ament_python: Python package (vs ament_cmake for C++)
# --dependencies: Automatically adds these to package.xml
```

**Output:**
```
going to create a new package
package name: my_robot_package
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['user <user@todo.todo>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: ['rclpy', 'std_msgs', 'sensor_msgs', 'geometry_msgs']
```

### Step 3: Examine the Generated Files

```bash
cd my_robot_package
tree
```

---

## Understanding package.xml

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>Humanoid robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

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

### Dependency Types

| Tag | Purpose | Example |
|-----|---------|---------|
| `<buildtool_depend>` | Tools needed to build | `ament_python`, `ament_cmake` |
| `<depend>` | Build + runtime dependency | `rclpy`, `std_msgs` |
| `<build_depend>` | Only needed for building | `rosidl_default_generators` |
| `<exec_depend>` | Only needed at runtime | `python3-numpy` |
| `<test_depend>` | Only needed for testing | `pytest`, `ament_lint` |

---

## Understanding setup.py

The `setup.py` file configures the Python package:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Humanoid robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
            'sensor_processor = my_robot_package.sensor_processor:main',
            'controller = my_robot_package.controller:main',
        ],
    },
)
```

### Entry Points Explained

Entry points define executable scripts:

```python
'console_scripts': [
    'executable_name = package_name.module_name:function_name',
]
```

This allows you to run:
```bash
ros2 run my_robot_package executable_name
```

---

## Creating a Complete Node

### Example: IMU Data Processor Node

Create `my_robot_package/sensor_processor.py`:

```python
#!/usr/bin/env python3
"""
Sensor Processor Node
Subscribes to IMU data, processes it, and publishes filtered output
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import numpy as np
from collections import deque

class SensorProcessor(Node):
    """
    Processes IMU sensor data with filtering and anomaly detection.
    """
    
    def __init__(self):
        super().__init__('sensor_processor')
        
        # Declare parameters
        self.declare_parameter('window_size', 10)
        self.declare_parameter('max_angular_velocity', 5.0)
        self.declare_parameter('publish_rate', 50.0)
        
        # Get parameters
        self.window_size = self.get_parameter('window_size').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10
        )
        
        # Create publishers
        self.filtered_pub = self.create_publisher(
            Vector3Stamped,
            'imu/angular_velocity_filtered',
            10
        )
        
        self.anomaly_pub = self.create_publisher(
            Vector3Stamped,
            'imu/anomalies',
            10
        )
        
        # Initialize moving average buffers
        self.angular_vel_buffer_x = deque(maxlen=self.window_size)
        self.angular_vel_buffer_y = deque(maxlen=self.window_size)
        self.angular_vel_buffer_z = deque(maxlen=self.window_size)
        
        # Statistics
        self.message_count = 0
        self.anomaly_count = 0
        
        # Create timer for logging statistics
        self.stats_timer = self.create_timer(1.0, self.log_statistics)
        
        self.get_logger().info(
            f'Sensor Processor initialized with window_size={self.window_size}'
        )
    
    def imu_callback(self, msg):
        """Process incoming IMU data"""
        self.message_count += 1
        
        # Extract angular velocity
        ang_vel_x = msg.angular_velocity.x
        ang_vel_y = msg.angular_velocity.y
        ang_vel_z = msg.angular_velocity.z
        
        # Check for anomalies
        if self.is_anomaly(ang_vel_x, ang_vel_y, ang_vel_z):
            self.anomaly_count += 1
            self.publish_anomaly(msg.header, ang_vel_x, ang_vel_y, ang_vel_z)
            return  # Skip filtering for anomalous data
        
        # Add to moving average buffers
        self.angular_vel_buffer_x.append(ang_vel_x)
        self.angular_vel_buffer_y.append(ang_vel_y)
        self.angular_vel_buffer_z.append(ang_vel_z)
        
        # Calculate filtered values (moving average)
        filtered_x = np.mean(self.angular_vel_buffer_x)
        filtered_y = np.mean(self.angular_vel_buffer_y)
        filtered_z = np.mean(self.angular_vel_buffer_z)
        
        # Publish filtered data
        self.publish_filtered(msg.header, filtered_x, filtered_y, filtered_z)
    
    def is_anomaly(self, x, y, z):
        """Check if angular velocity exceeds threshold"""
        magnitude = np.sqrt(x**2 + y**2 + z**2)
        return magnitude > self.max_angular_vel
    
    def publish_filtered(self, header, x, y, z):
        """Publish filtered angular velocity"""
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        self.filtered_pub.publish(msg)
    
    def publish_anomaly(self, header, x, y, z):
        """Publish detected anomaly"""
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        self.anomaly_pub.publish(msg)
        
        self.get_logger().warn(
            f'Anomaly detected: magnitude={np.sqrt(x**2 + y**2 + z**2):.2f} rad/s'
        )
    
    def log_statistics(self):
        """Log processing statistics"""
        if self.message_count > 0:
            anomaly_rate = (self.anomaly_count / self.message_count) * 100
            self.get_logger().info(
                f'Processed {self.message_count} messages, '
                f'{self.anomaly_count} anomalies ({anomaly_rate:.1f}%)'
            )

def main(args=None):
    rclpy.init(args=args)
    sensor_processor = SensorProcessor()
    
    try:
        rclpy.spin(sensor_processor)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Add Entry Point to setup.py

```python
entry_points={
    'console_scripts': [
        'sensor_processor = my_robot_package.sensor_processor:main',
    ],
},
```

---

## Building and Installing the Package

### Step 1: Build the Package

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build specific package
colcon build --packages-select my_robot_package

# Or build all packages
colcon build

# Build with symlink install (for faster Python development)
colcon build --symlink-install
```

### Step 2: Source the Workspace

```bash
# Source the setup file
source install/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: Run the Node

```bash
# Run the node
ros2 run my_robot_package sensor_processor

# Run with parameters
ros2 run my_robot_package sensor_processor --ros-args \
  -p window_size:=20 \
  -p max_angular_velocity:=10.0
```

---

## Creating Configuration Files

### Parameter YAML File

Create `config/sensor_params.yaml`:

```yaml
sensor_processor:
  ros__parameters:
    window_size: 15
    max_angular_velocity: 8.0
    publish_rate: 100.0
    
    # Logging configuration
    log_level: INFO
```

### Using Parameters from File

```bash
ros2 run my_robot_package sensor_processor --ros-args \
  --params-file src/my_robot_package/config/sensor_params.yaml
```

---

## Creating Launch Files

Launch files allow you to start multiple nodes with configurations.

### Python Launch File

Create `launch/robot_bringup.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_package')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = LaunchConfiguration('config_file', 
                                     default=os.path.join(pkg_dir, 'config', 'sensor_params.yaml'))
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(pkg_dir, 'config', 'sensor_params.yaml'),
            description='Path to parameter file'
        ),
        
        # IMU Publisher Node
        Node(
            package='my_robot_package',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Sensor Processor Node
        Node(
            package='my_robot_package',
            executable='sensor_processor',
            name='sensor_processor',
            output='screen',
            parameters=[config_file, {'use_sim_time': use_sim_time}]
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(os.path.join(pkg_dir, 'urdf', 'robot.urdf')).read(),
                'use_sim_time': use_sim_time
            }]
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'robot.rviz')],
            condition=LaunchConfiguration('rviz', default='true')
        ),
    ])
```

### Running the Launch File

```bash
# Run the launch file
ros2 launch my_robot_package robot_bringup.launch.py

# With arguments
ros2 launch my_robot_package robot_bringup.launch.py use_sim_time:=true rviz:=false
```

---

## Package Organization Best Practices

### Directory Structure for Large Projects

```
my_robot_package/
├── my_robot_package/
│   ├── __init__.py
│   ├── nodes/              # Node implementations
│   │   ├── __init__.py
│   │   ├── sensor_node.py
│   │   └── controller_node.py
│   ├── utils/              # Utility modules
│   │   ├── __init__.py
│   │   ├── filters.py
│   │   └── transforms.py
│   └── interfaces/         # Custom message/service definitions (if any)
│       └── __init__.py
├── launch/
│   ├── robot_bringup.launch.py
│   └── simulation.launch.py
├── config/
│   ├── sensor_params.yaml
│   └── controller_params.yaml
├── urdf/
│   ├── robot.urdf
│   └── robot.urdf.xacro
├── rviz/
│   └── robot.rviz
└── test/
    ├── test_sensor_node.py
    └── test_controller.py
```

---

## Testing Your Package

### Unit Test Example

Create `test/test_sensor_processor.py`:

```python
import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from my_robot_package.sensor_processor import SensorProcessor

class TestSensorProcessor(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    
    def setUp(self):
        self.node = SensorProcessor()
    
    def tearDown(self):
        self.node.destroy_node()
    
    def test_anomaly_detection(self):
        """Test that anomalies are correctly detected"""
        # Normal angular velocity
        self.assertFalse(self.node.is_anomaly(0.1, 0.1, 0.1))
        
        # Anomalous angular velocity
        self.assertTrue(self.node.is_anomaly(10.0, 0.0, 0.0))
    
    def test_parameter_initialization(self):
        """Test that parameters are correctly initialized"""
        self.assertEqual(self.node.window_size, 10)
        self.assertEqual(self.node.max_angular_vel, 5.0)

if __name__ == '__main__':
    unittest.main()
```

### Running Tests

```bash
# Run all tests in package
colcon test --packages-select my_robot_package

# View test results
colcon test-result --verbose
```

---

## Common Package Commands

```bash
# List all packages
ros2 pkg list

# Get package information
ros2 pkg prefix my_robot_package

# List package executables
ros2 pkg executables my_robot_package

# Create package with dependencies
ros2 pkg create --build-type ament_python my_new_package \
  --dependencies rclpy std_msgs

# Build specific package
colcon build --packages-select my_robot_package

# Clean build
rm -rf build install log
colcon build
```

---

## Troubleshooting

> [!TIP]
> **Issue:** `ModuleNotFoundError: No module named 'my_robot_package'`
> 
> **Solution:**
> ```bash
> # Rebuild with symlink install
> colcon build --symlink-install
> source install/setup.bash
> ```

> [!TIP]
> **Issue:** Executable not found after adding to setup.py
> 
> **Solution:**
> ```bash
> # Rebuild the package
> colcon build --packages-select my_robot_package
> source install/setup.bash
> ```

> [!TIP]
> **Issue:** Launch file not found
> 
> **Solution:**
> Ensure launch files are included in `setup.py`:
> ```python
> (os.path.join('share', package_name, 'launch'),
>     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
> ```

---

## Summary

- ROS 2 packages organize code, configurations, and resources
- `package.xml` defines metadata and dependencies
- `setup.py` configures Python package installation
- Entry points make nodes executable with `ros2 run`
- Launch files start multiple nodes with configurations
- Proper organization scales to large projects

In the next chapter, we'll dive deeper into launch files and parameter management for complex robot systems.

---

## Further Reading

- [ROS 2 Package Creation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [Python Setup.py Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)
- [Colcon Build Tool](https://colcon.readthedocs.io/)
