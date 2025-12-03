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
| **Python Version** | Python 2.7 | Python 3.x |
| **Build System** | catkin | ament/colcon |
| **Discovery** | Master (roscore) | Auto-discovery via DDS |
| **Message Format** | Custom | DDS (RTPS) |

### Why the Change?

**ROS 1 Limitations:**
1. **Single Point of Failure**: roscore master must always be running
2. **Network Configuration**: Complex setup for distributed systems
3. **No Real-time**: Cannot guarantee timing constraints
4. **Limited Security**: No authentication or encryption
5. **Python 2**: End of life, security vulnerabilities

**ROS 2 Advantages:**
1. **No Master**: Fully distributed, no single point of failure
2. **Auto-discovery**: Nodes find each other automatically
3. **Real-time Ready**: Can be used with RTOS for hard real-time
4. **Secure**: DDS security plugins (authentication, encryption)
5. **Modern**: C++14/17, Python 3, current best practices

## ROS 2 Architectural Layers

Understanding the stack from bottom to top:

```
┌────────────────────────────────────────────┐
│     Application Layer (Your Code)         │
│  Publishers, Subscribers, Services, etc.  │
└─────────────────┬──────────────────────────┘
                  │
┌─────────────────▼──────────────────────────┐
│     ROS 2 Client Libraries (rclpy/rclcpp) │
│  Pythonic/C++ API for node functionality  │
└─────────────────┬──────────────────────────┘
                  │
┌─────────────────▼──────────────────────────┐
│          ROS 2 Core (rcl)                  │
│  Language-agnostic core functionality     │
│  Node lifecycle, Time, Graph API          │
└─────────────────┬──────────────────────────┘
                  │
┌─────────────────▼──────────────────────────┐
│     ROS Middleware Interface (rmw)         │
│  Abstraction layer over DDS                │
└─────────────────┬──────────────────────────┘
                  │
┌─────────────────▼──────────────────────────┐
│     DDS Implementation                     │
│  Fast DDS (default), Cyclone DDS, RTI     │
└─────────────────┬──────────────────────────┘
                  │
┌─────────────────▼──────────────────────────┐
│     Operating System / Network             │
│  UDP/TCP, Shared Memory                    │
└────────────────────────────────────────────┘
```

### DDS: The Foundation of ROS 2

**Data Distribution Service (DDS)** is an OMG standard for real-time, scalable data exchange.

**Why DDS?**
- **Industry Standard**: Used in aerospace, defense, automotive
- **Proven**: Battle-tested in mission-critical systems
- **Real-time**: Designed for hard real-time systems
- **Scalable**: Supports thousands of nodes
- **Secure**: Built-in security (authentication, encryption, access control)

**DDS Concepts Mapping to ROS 2:**

| DDS Concept | ROS 2 Equivalent | Description |
|-------------|------------------|-------------|
| **Participant** | Node | An entity that publishes/subscribes |
| **Topic** | Topic | Named channel for data exchange |
| **DataWriter** | Publisher | Sends data to a topic |
| **DataReader** | Subscriber | Receives data from a topic |
| **QoS Policies** | QoS Profiles | Quality of Service settings |
| **Discovery** | Auto-discovery | Finding other participants |

**DDS Implementations Available:**
- **Fast DDS** (eProsima): Default, feature-rich
- **Cyclone DDS** (Eclipse): Lightweight, fast
- **RTI Connext DDS**: Commercial, highest performance
- **GurumDDS**: Real-time focused

## Core ROS 2 Concepts

### 1. Nodes

**Definition:** A node is an executable that performs computation by processing data from sensors, controlling actuators, or computing outputs.

**Characteristics:**
- Independent process (can crash without affecting others)
- Communicates via topics, services, or actions
- Can publish and subscribe to multiple topics
- Can provide and call services
- Runs in its own thread/process

**When to Create a New Node:**
```
Good Separation:
✓ Camera driver node (hardware interface)
✓ Image processing node (computation)
✓ Object detection node (AI inference)
✓ Controller node (decision making)

Poor Separation:
✗ Everything in one monolithic node
✗ Too many tiny nodes (overhead)
```

**Node Lifecycle:**

```
     ┌──────────┐
     │Unconfigured│
     └─────┬──────┘
           │ configure()
     ┌─────▼──────┐
     │ Inactive   │◄────┐
     └─────┬──────┘     │
           │ activate() │ deactivate()
     ┌─────▼──────┐     │
     │  Active    │─────┘
     └─────┬──────┘
           │ cleanup()
     ┌─────▼──────┐
     │ Finalized  │
     └────────────┘
```

### 2. Topics: Publish-Subscribe Pattern

**Use Case:** Continuous data streams (sensor data, robot state)

**Characteristics:**
- **Asynchronous**: Publishers don't wait for subscribers
- **Many-to-many**: Multiple publishers/subscribers allowed
- **Unidirectional**: Data flows one way
- **Best effort**: No guaranteed delivery (configurable with QoS)

**Example Scenarios:**
```
✓ Camera publishing images (continuous stream)
✓ Lidar publishing point clouds
✓ Robot publishing joint states
✓ IMU publishing orientation data
```

**Python Example:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(
            String,      # Message type
            'topic',     # Topic name
            10           # Queue size (QoS depth)
        )
        
        # Timer for periodic publishing
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
```

**Subscriber Example:**

```python
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Create subscription
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### 3. Services: Request-Response Pattern

**Use Case:** Occasional transactions, computations on demand

**Characteristics:**
- **Synchronous**: Client waits for server response
- **One-to-one**: Single client request, single server response
- **Bidirectional**: Request and response
- **Blocking**: Client blocks until response (or timeout)

**Example Scenarios:**
```
✓ Trigger calibration routine
✓ Request path planning computation
✓ Get current battery level
✓ Save configuration to disk
```

**Service Definition (.srv file):**
```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

**Server Implementation:**

```python
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        
        # Create service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a} b={request.b}'
        )
        return response
```

**Client Implementation:**

```python
class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        
        # Create client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        
        # Async call
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()
```

### 4. Actions: Goal-Oriented Tasks with Feedback

**Use Case:** Long-running tasks that need feedback and can be cancelled

**Characteristics:**
- **Asynchronous**: Client doesn't block
- **Feedback**: Server sends periodic updates
- **Cancellable**: Client can cancel goal mid-execution
- **Result**: Final outcome when goal completes

**Example Scenarios:**
```
✓ Navigate to goal position (with progress updates)
✓ Pick and place object (phases: approach, grasp, move, release)
✓ Charge battery (with percentage updates)
✓ Execute trajectory (with waypoint completion)
```

**Action Structure:**
```
Goal:    What you want to achieve
Feedback: Periodic updates during execution
Result:  Final outcome
```

**Action Definition (.action file):**
```
# Fibonacci.action
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

**Action Server:**

```python
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            # Publish feedback
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + 
                feedback_msg.partial_sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

### 5. Parameters: Runtime Configuration

**Use Case:** Configure node behavior without code changes

**Characteristics:**
- **Dynamic**: Can be changed at runtime
- **Typed**: int, double, string, bool, byte array
- **Declared**: Must be declared before use (in ROS 2)
- **Persistent**: Can be saved to YAML files

**Parameter Declaration:**

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters
        self.declare_parameter('my_param', 'default_value')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('robot_name', 'my_robot')
        
        # Get parameter values
        my_param = self.get_parameter('my_param').value
        max_speed = self.get_parameter('max_speed').value
        
        self.get_logger().info(f'my_param: {my_param}')
        self.get_logger().info(f'max_speed: {max_speed}')
```

## Quality of Service (QoS) Policies

QoS policies control how messages are delivered. Critical for reliable robot systems!

### Common QoS Policies

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | Best Effort, Reliable | Reliable for commands, Best Effort for sensor streams |
| **Durability** | Volatile, Transient Local | Transient for late-joining subscribers |
| **History** | Keep Last, Keep All | Keep Last(10) typical for topics |
| **Depth** | 1-N | Queue size for messages |
| **Deadline** | Duration | Detect if publisher stopped |
| **Liveliness** | Automatic, Manual | Detect if node crashed |

### QoS Profiles

Pre-configured profiles for common scenarios:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Sensor data (fast, can lose occasional messages)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Commands (must be delivered, keep recent)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Create publisher with custom QoS
self.publisher = self.create_publisher(
    String,
    'topic',
    sensor_qos  # Apply QoS profile
)
```

**QoS Matching Rules:**
- Publishers and subscribers must have compatible QoS
- If incompatible, connection will fail silently!
- Use `ros2 topic info -v` to check QoS settings

## Communication Patterns Decision Tree

```
Need to send data?
│
├─ Continuous stream? → TOPIC
│  └─ Examples: sensor data, robot state
│
├─ Request computation? → SERVICE
│  └─ Examples: inverse kinematics, path planning
│
└─ Long-running task? → ACTION
   └─ Examples: navigation, manipulation trajectory
```

## Common Pitfalls and Best Practices

### ❌ Common Mistakes

1. **Using Services for Continuous Data**
   - Services block—bad for high-frequency data
   - Use topics instead

2. **Not Setting Proper QoS**
   - Default QoS may not match your needs
   - Sensor data: Best Effort
   - Commands: Reliable

3. **Too Many Nodes**
   - Each node has overhead
   - Group related functionality

4. **Not Handling Late Subscribers**
   - Use Transient Local durability if needed
   - Or implement latching behavior

### ✅ Best Practices

1. **One Concern per Node**
   - Sensor driver separate from processing
   - Easy to test and debug

2. **Use Namespaces**
   - `/robot1/camera/image`
   - Enables multi-robot systems

3. **Declare All Parameters**
   - Makes configuration explicit
   - Enables validation

4. **Check QoS Compatibility**
   - Verify before deployment
   - Use `ros2 topic info -v`

## Debugging and Introspection Tools

**Check Running Nodes:**
```bash
ros2 node list
ros2 node info /my_node
```

**Inspect Topics:**
```bash
ros2 topic list
ros2 topic echo /topic_name
ros2 topic hz /topic_name      # Check frequency
ros2 topic info -v /topic_name # Verbose (shows QoS)
```

**Visualize Node Graph:**
```bash
rqt_graph  # GUI tool showing node connections
```

**Monitor System:**
```bash
ros2 run rqt_console rqt_console  # Log viewer
ros2 run rqt_plot rqt_plot          # Real-time plotting
```

## Summary

ROS 2 architecture provides:
- **Distributed computing** without a central master
- **Flexible communication** via topics, services, and actions
- **Real-time capabilities** through DDS
- **Security** via DDS security plugins
- **Scalability** from single robots to large fleets

Understanding these concepts is fundamental to building robust Physical AI systems.

## Key Takeaways

✓ ROS 2 uses DDS for distributed, real-time communication  
✓ Nodes are independent processes that communicate via topics/services/actions  
✓ Topics = continuous streams, Services = request-response, Actions = long-running tasks  
✓ QoS policies control message delivery characteristics  
✓ Choose communication pattern based on use case requirements  

## Discussion Questions

1. Why might you choose topics over services for robot odometry data?
2. In what scenarios would an action be preferable to a service?
3. How does removing the master node (roscore) improve system robustness?
4. What QoS settings would you use for emergency stop commands? Why?

##Practice Exercise

**Design a Node Graph:**

Create a node architecture diagram for an autonomous mobile robot with:
- Camera for object detection
- Lidar for obstacle avoidance
- Odometry for position tracking
- Navigation planner
- Motor controllers

For each connection, specify:
- Communication type (topic/service/action)
- Message type
- QoS policy
- Justification

---

**Next:** Proceed to [Environment Setup](./ros2-environment.md) to install and configure your ROS 2 development environment.