---
id: launch-files-parameters
title: Launch Files and Parameter Management
slug: /module-02/launch-files-parameters
---

# Launch Files and Parameter Management

## Introduction

**Launch files** in ROS 2 allow you to start multiple nodes, set parameters, and configure your entire robot system with a single command. **Parameters** provide runtime configuration without modifying code. Together, they enable flexible, reusable robot applications.

## Why Launch Files Matter

Instead of running nodes individually:
```bash
ros2 run package1 node1
ros2 run package2 node2 --ros-args -p param:=value
ros2 run package3 node3
```

Use a single launch file:
```bash
ros2 launch my_robot_package robot.launch.py
```

---

## Python Launch Files (Recommended)

ROS 2 uses Python-based launch files for maximum flexibility.

### Basic Launch File Structure

Create `launch/basic_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='sensor_node',
            name='sensor_node',
            output='screen'
        ),
        Node(
            package='my_robot_package',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
    ])
```

### Running the Launch File

```bash
ros2 launch my_robot_package basic_robot.launch.py
```

---

## Launch File Components

### 1. Nodes

```python
Node(
    package='package_name',           # Package containing the executable
    executable='node_executable',     # Executable name from setup.py
    name='custom_node_name',          # Override node name
    namespace='robot1',               # Node namespace
    output='screen',                  # 'screen' or 'log'
    parameters=[{'param': value}],    # Node parameters
    remappings=[('old_topic', 'new_topic')],  # Topic remapping
    arguments=['--arg1', 'value1'],   # Command-line arguments
)
```

### 2. Launch Arguments

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Use argument value
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='my_robot_package',
            executable='sensor_node',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
```

**Usage:**
```bash
ros2 launch my_robot_package robot.launch.py use_sim_time:=true
```

### 3. Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('other_package')
    
    # Include another launch file
    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'other.launch.py')
        ),
        launch_arguments={
            'param1': 'value1',
            'param2': 'value2'
        }.items()
    )
    
    return LaunchDescription([other_launch])
```

### 4. Conditional Execution

```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    return LaunchDescription([
        # Only launch if use_rviz is true
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz)
        ),
        
        # Only launch if use_rviz is false
        Node(
            package='my_package',
            executable='headless_node',
            condition=UnlessCondition(use_rviz)
        ),
    ])
```

---

## Complete Humanoid Robot Launch File

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('humanoid_robot').find('humanoid_robot')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    robot_name = LaunchConfiguration('robot_name', default='humanoid_01')
    
    # File paths
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'humanoid.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'humanoid.rviz'])
    params_file = PathJoinSubstitution([pkg_share, 'config', 'robot_params.yaml'])
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='humanoid_01',
            description='Robot namespace'
        ),
        
        # Robot State Publisher (publishes TF from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file]),
                'use_sim_time': use_sim_time
            }]
        ),
        
        # Joint State Publisher (for testing without hardware)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=robot_name,
            condition=IfCondition(LaunchConfiguration('use_gui', default='false'))
        ),
        
        # IMU Driver
        Node(
            package='humanoid_robot',
            executable='imu_driver',
            name='imu_driver',
            namespace=robot_name,
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),
        
        # Sensor Processor
        Node(
            package='humanoid_robot',
            executable='sensor_processor',
            name='sensor_processor',
            namespace=robot_name,
            output='screen',
            parameters=[params_file]
        ),
        
        # Balance Controller (delayed start to wait for sensors)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='humanoid_robot',
                    executable='balance_controller',
                    name='balance_controller',
                    namespace=robot_name,
                    output='screen',
                    parameters=[params_file]
                )
            ]
        ),
        
        # RViz (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
```

---

## Parameter Management

### 1. Declaring Parameters in Nodes

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters with defaults and descriptions
        self.declare_parameter('max_velocity', 1.0, 
            descriptor=ParameterDescriptor(
                description='Maximum velocity in m/s',
                type=ParameterType.PARAMETER_DOUBLE,
                read_only=False
            )
        )
        
        self.declare_parameter('robot_name', 'robot_01')
        self.declare_parameter('enable_safety', True)
        
        # Get parameter values
        self.max_vel = self.get_parameter('max_velocity').value
        self.robot_name = self.get_parameter('robot_name').value
        self.safety_enabled = self.get_parameter('enable_safety').value
        
        # Add parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, params):
        """Called when parameters are changed at runtime"""
        for param in params:
            if param.name == 'max_velocity':
                self.max_vel = param.value
                self.get_logger().info(f'Max velocity updated to {param.value}')
        return SetParametersResult(successful=True)
```

### 2. YAML Parameter Files

Create `config/robot_params.yaml`:

```yaml
# IMU Driver Parameters
imu_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baud_rate: 115200
    frame_id: "imu_link"
    publish_rate: 100.0
    
    # Calibration offsets
    gyro_offset_x: 0.001
    gyro_offset_y: -0.002
    gyro_offset_z: 0.0
    
    accel_offset_x: 0.0
    accel_offset_y: 0.0
    accel_offset_z: 0.0

# Sensor Processor Parameters
sensor_processor:
  ros__parameters:
    window_size: 10
    max_angular_velocity: 5.0
    anomaly_threshold: 3.0
    filter_type: "moving_average"  # or "kalman", "lowpass"

# Balance Controller Parameters
balance_controller:
  ros__parameters:
    # PID gains
    kp_pitch: 0.5
    ki_pitch: 0.01
    kd_pitch: 0.1
    
    kp_roll: 0.5
    ki_roll: 0.01
    kd_roll: 0.1
    
    # Control limits
    max_torque: 50.0
    control_rate: 200.0
    
    # Safety
    enable_safety_limits: true
    max_tilt_angle: 0.5  # radians
```

### 3. Loading Parameters in Launch Files

```python
# Method 1: From YAML file
Node(
    package='my_package',
    executable='my_node',
    parameters=[params_file]
)

# Method 2: Inline parameters
Node(
    package='my_package',
    executable='my_node',
    parameters=[{
        'param1': 'value1',
        'param2': 42,
        'param3': True
    }]
)

# Method 3: Mix of file and inline
Node(
    package='my_package',
    executable='my_node',
    parameters=[
        params_file,
        {'override_param': 'new_value'}
    ]
)

# Method 4: Using launch arguments
Node(
    package='my_package',
    executable='my_node',
    parameters=[{
        'use_sim_time': use_sim_time,
        'robot_name': robot_name
    }]
)
```

### 4. Runtime Parameter Management

```bash
# List all parameters for a node
ros2 param list /my_node

# Get parameter value
ros2 param get /my_node max_velocity

# Set parameter value
ros2 param set /my_node max_velocity 2.0

# Dump all parameters to file
ros2 param dump /my_node > my_node_params.yaml

# Load parameters from file
ros2 param load /my_node my_node_params.yaml
```

---

## Advanced Launch Techniques

### 1. Event Handlers

```python
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown

def generate_launch_description():
    sensor_node = Node(
        package='my_package',
        executable='sensor_node',
        name='sensor_node'
    )
    
    controller_node = Node(
        package='my_package',
        executable='controller_node',
        name='controller_node'
    )
    
    return LaunchDescription([
        sensor_node,
        
        # Start controller only after sensor node starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=sensor_node,
                on_start=[controller_node]
            )
        ),
        
        # Shutdown everything if sensor node exits
        RegisterEventHandler(
            OnProcessExit(
                target_action=sensor_node,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        ),
    ])
```

### 2. Composable Nodes (for performance)

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='my_package',
                plugin='my_package::SensorNode',
                name='sensor_node',
                parameters=[{'param': 'value'}]
            ),
            ComposableNode(
                package='my_package',
                plugin='my_package::ProcessorNode',
                name='processor_node'
            ),
        ],
        output='screen',
    )
    
    return LaunchDescription([container])
```

### 3. Environment Variables

```python
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', 
                              '[{severity}] [{name}]: {message}'),
        
        # Your nodes here
    ])
```

---

## Multi-Robot Launch Example

```python
def generate_launch_description():
    robots = []
    
    # Launch 3 robots with different namespaces
    for i in range(1, 4):
        robot_name = f'robot_{i}'
        
        robot_nodes = [
            Node(
                package='humanoid_robot',
                executable='sensor_node',
                name='sensor_node',
                namespace=robot_name,
                parameters=[{
                    'robot_id': i,
                    'x_offset': i * 2.0
                }]
            ),
            Node(
                package='humanoid_robot',
                executable='controller',
                name='controller',
                namespace=robot_name,
                parameters=[{'robot_id': i}]
            ),
        ]
        
        robots.extend(robot_nodes)
    
    return LaunchDescription(robots)
```

---

## Debugging Launch Files

### 1. Print Launch Configuration

```python
from launch.actions import LogInfo

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default='robot_01')
    
    return LaunchDescription([
        LogInfo(msg=['Robot name: ', robot_name]),
        
        # Your nodes...
    ])
```

### 2. Verbose Output

```bash
# Run with verbose output
ros2 launch my_package robot.launch.py --debug

# Show launch file structure
ros2 launch my_package robot.launch.py --show-args
ros2 launch my_package robot.launch.py --show-all-subprocesses-output
```

---

## Best Practices

### ✅ DO:

1. **Use descriptive argument names**
   ```python
   DeclareLaunchArgument('use_sim_time', ...)  # Good
   DeclareLaunchArgument('sim', ...)           # Bad
   ```

2. **Provide default values and descriptions**
   ```python
   DeclareLaunchArgument(
       'max_velocity',
       default_value='1.0',
       description='Maximum robot velocity in m/s'
   )
   ```

3. **Organize parameters in YAML files**
   - Easier to modify without changing code
   - Version control friendly
   - Reusable across different robots

4. **Use namespaces for multi-robot systems**
   ```python
   namespace='robot_01'
   ```

5. **Add parameter validation in nodes**
   ```python
   if self.max_vel <= 0:
       raise ValueError('max_velocity must be positive')
   ```

### ❌ DON'T:

1. **Don't hardcode values in launch files**
   ```python
   # Bad
   parameters=[{'port': '/dev/ttyUSB0'}]
   
   # Good
   parameters=[params_file]
   ```

2. **Don't ignore parameter types**
   ```python
   # Bad: passing string when int expected
   parameters=[{'count': '10'}]
   
   # Good
   parameters=[{'count': 10}]
   ```

3. **Don't launch too many nodes in one file**
   - Split into multiple launch files
   - Use IncludeLaunchDescription

---

## Summary

- Launch files start multiple nodes with single command
- Python launch files provide maximum flexibility
- Parameters enable runtime configuration
- YAML files organize parameters cleanly
- Event handlers coordinate node startup
- Namespaces enable multi-robot systems

In the next chapter, we'll learn how to bridge Python AI agents with ROS 2 controllers for intelligent robot behavior.

---

## Further Reading

- [ROS 2 Launch Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 Parameters](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html)
- [Launch File Examples](https://github.com/ros2/launch_ros/tree/humble/launch_ros/examples)
