---
id: jetson-deployment
title: Deploying to Jetson Hardware
slug: /module-04/jetson-deployment
---

# Deploying to Jetson Hardware

## Introduction

**NVIDIA Jetson** is the leading edge AI platform for robotics. This chapter covers deploying ROS 2 and AI models to Jetson Orin for real-time humanoid robot control.

## Jetson Platform Overview

| Model | GPU | CPU | RAM | Power | Price | Use Case |
|-------|-----|-----|-----|-------|-------|----------|
| **Orin Nano** | 1024 CUDA cores | 6-core ARM | 8GB | 7-15W | $499 | Research, prototyping |
| **Orin NX** | 1024 CUDA cores | 8-core ARM | 16GB | 10-25W | $699 | Production robots |
| **AGX Orin** | 2048 CUDA cores | 12-core ARM | 64GB | 15-60W | $1,999 | High-performance |

---

## Initial Setup

### 1. Flash JetPack

```bash
# Download NVIDIA SDK Manager
# https://developer.nvidia.com/sdk-manager

# Flash JetPack 5.1.2 (includes Ubuntu 20.04, CUDA, cuDNN, TensorRT)
# Follow GUI instructions

# After flashing, SSH into Jetson
ssh nvidia@<jetson-ip>
```

### 2. Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Isaac ROS

```bash
# Install dependencies
sudo apt-get install -y python3-pip
pip3 install -U jetson-stats

# Clone Isaac ROS
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Install Isaac ROS packages
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Optimizing AI Models for Jetson

### 1. TensorRT Optimization

```python
#!/usr/bin/env python3
"""
Convert PyTorch model to TensorRT for Jetson deployment.
"""

import torch
import tensorrt as trt
from torch2trt import torch2trt

class PolicyNetwork(torch.nn.Module):
    """Example policy network"""
    def __init__(self):
        super().__init__()
        self.fc1 = torch.nn.Linear(30, 256)
        self.fc2 = torch.nn.Linear(256, 256)
        self.fc3 = torch.nn.Linear(256, 12)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return torch.tanh(self.fc3(x))

def convert_to_tensorrt(model_path, output_path):
    """
    Convert PyTorch model to TensorRT.
    
    Args:
        model_path: Path to PyTorch model
        output_path: Path to save TensorRT engine
    """
    # Load PyTorch model
    model = PolicyNetwork()
    model.load_state_dict(torch.load(model_path))
    model.eval()
    model.cuda()
    
    # Create example input
    x = torch.ones((1, 30)).cuda()
    
    # Convert to TensorRT
    model_trt = torch2trt(
        model,
        [x],
        fp16_mode=True,  # Use FP16 for speed
        max_batch_size=1
    )
    
    # Save TensorRT model
    torch.save(model_trt.state_dict(), output_path)
    
    print(f'TensorRT model saved to {output_path}')
    
    # Benchmark
    import time
    
    # PyTorch inference
    start = time.time()
    for _ in range(1000):
        with torch.no_grad():
            _ = model(x)
    torch.cuda.synchronize()
    pytorch_time = time.time() - start
    
    # TensorRT inference
    start = time.time()
    for _ in range(1000):
        _ = model_trt(x)
    torch.cuda.synchronize()
    tensorrt_time = time.time() - start
    
    print(f'PyTorch: {pytorch_time:.3f}s')
    print(f'TensorRT: {tensorrt_time:.3f}s')
    print(f'Speedup: {pytorch_time/tensorrt_time:.2f}x')

if __name__ == '__main__':
    convert_to_tensorrt('policy.pth', 'policy_trt.pth')
```

### 2. Quantization

```python
import torch
from torch.quantization import quantize_dynamic

def quantize_model(model_path, output_path):
    """
    Quantize model to INT8 for faster inference.
    
    Args:
        model_path: Path to original model
        output_path: Path to save quantized model
    """
    # Load model
    model = PolicyNetwork()
    model.load_state_dict(torch.load(model_path))
    model.eval()
    
    # Quantize
    quantized_model = quantize_dynamic(
        model,
        {torch.nn.Linear},  # Quantize Linear layers
        dtype=torch.qint8
    )
    
    # Save
    torch.save(quantized_model.state_dict(), output_path)
    
    print(f'Quantized model saved to {output_path}')
```

---

## Real-Time Control Loop

```python
#!/usr/bin/env python3
"""
Real-time control loop on Jetson.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import torch
from torch2trt import TRTModule
import numpy as np
import time

class JetsonController(Node):
    """
    Real-time robot controller running on Jetson.
    """
    
    def __init__(self):
        super().__init__('jetson_controller')
        
        # Load TensorRT model
        self.model = TRTModule()
        self.model.load_state_dict(torch.load('policy_trt.pth'))
        self.model.eval()
        
        # State
        self.observation = np.zeros(30, dtype=np.float32)
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        
        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )
        
        # Control loop (50 Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Performance monitoring
        self.loop_times = []
        
        self.get_logger().info('Jetson Controller initialized')
    
    def joint_callback(self, msg):
        """Update joint state"""
        self.observation[0:12] = msg.position
        self.observation[12:24] = msg.velocity
    
    def imu_callback(self, msg):
        """Update IMU data"""
        self.observation[24:27] = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        self.observation[27:30] = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
    
    def control_loop(self):
        """Main control loop - runs at 50 Hz"""
        start_time = time.time()
        
        # Prepare input
        obs_tensor = torch.from_numpy(self.observation).unsqueeze(0).cuda()
        
        # Inference
        with torch.no_grad():
            action_tensor = self.model(obs_tensor)
        
        # Convert to numpy
        action = action_tensor.cpu().numpy().flatten()
        
        # Publish command
        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.cmd_pub.publish(msg)
        
        # Monitor performance
        loop_time = time.time() - start_time
        self.loop_times.append(loop_time)
        
        if len(self.loop_times) >= 100:
            avg_time = np.mean(self.loop_times)
            max_time = np.max(self.loop_times)
            self.get_logger().info(
                f'Control loop: avg={avg_time*1000:.2f}ms, max={max_time*1000:.2f}ms'
            )
            self.loop_times = []

def main(args=None):
    rclpy.init(args=args)
    controller = JetsonController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Power Management

```python
#!/usr/bin/env python3
"""
Power management for Jetson.
"""

import subprocess

class PowerManager:
    """
    Manages Jetson power modes.
    """
    
    def __init__(self):
        self.modes = {
            'max_performance': 0,  # 60W
            'balanced': 1,          # 30W
            'low_power': 2          # 15W
        }
    
    def set_power_mode(self, mode):
        """
        Set power mode.
        
        Args:
            mode: 'max_performance', 'balanced', or 'low_power'
        """
        if mode not in self.modes:
            raise ValueError(f'Invalid mode: {mode}')
        
        mode_id = self.modes[mode]
        
        # Set nvpmodel
        subprocess.run(['sudo', 'nvpmodel', '-m', str(mode_id)])
        
        # Set jetson_clocks for max performance
        if mode == 'max_performance':
            subprocess.run(['sudo', 'jetson_clocks'])
        
        print(f'Power mode set to: {mode}')
    
    def get_power_usage(self):
        """Get current power usage"""
        result = subprocess.run(
            ['tegrastats', '--interval', '1000'],
            capture_output=True,
            text=True,
            timeout=2
        )
        
        # Parse output for power consumption
        # ...
        
        return power_watts
```

---

## Deployment Checklist

### ✅ Pre-Deployment

- [ ] Test model in simulation
- [ ] Optimize model with TensorRT
- [ ] Benchmark inference time
- [ ] Test on Jetson dev kit
- [ ] Verify real-time performance (< 20ms loop)
- [ ] Test power consumption
- [ ] Implement safety checks

### ✅ Deployment

- [ ] Flash production Jetson
- [ ] Install all dependencies
- [ ] Copy optimized models
- [ ] Configure systemd service for auto-start
- [ ] Set up remote monitoring
- [ ] Test emergency stop
- [ ] Document deployment process

### ✅ Post-Deployment

- [ ] Monitor performance metrics
- [ ] Log errors and crashes
- [ ] Collect real-world data
- [ ] Plan model updates
- [ ] Schedule maintenance

---

## Systemd Service for Auto-Start

```bash
# Create service file
sudo nano /etc/systemd/system/robot-controller.service
```

```ini
[Unit]
Description=Humanoid Robot Controller
After=network.target

[Service]
Type=simple
User=nvidia
WorkingDirectory=/home/nvidia/ros2_ws
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 run robot_control jetson_controller"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Enable and start service
sudo systemctl enable robot-controller
sudo systemctl start robot-controller

# Check status
sudo systemctl status robot-controller

# View logs
sudo journalctl -u robot-controller -f
```

---

## Summary

- **Jetson Orin** is the platform of choice for edge AI robotics
- **TensorRT** provides 2-5x speedup over PyTorch
- **Real-time control** requires optimized inference (< 20ms)
- **Power management** balances performance and battery life
- **Systemd services** enable reliable auto-start

Jetson deployment brings AI models from simulation to real robots.

---

## Further Reading

- [Jetson Orin Documentation](https://developer.nvidia.com/embedded/jetson-orin)
- [Isaac ROS](https://nvidia-isaac-ros.github.io/)
- [TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)
- [Jetson Community](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)
