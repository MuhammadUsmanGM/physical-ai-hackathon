---
id: ros2-environment
title: Setting Up Your ROS 2 Development Environment
sidebar_label: Environment Setup
---

# Setting Up Your ROS 2 Development Environment

A properly configured development environment is essential for productive ROS 2 development. This chapter guides you through installing ROS 2 Humble, configuring your workspace, and setting up development tools.

## Prerequisites

Before installing ROS 2, ensure your system meets these requirements:

**Operating System:**
- Ubuntu 22.04 LTS (Jammy Jellyfish) - **Recommended**
- Ubuntu 20.04 LTS (with ROS 2 Galactic)
- Other Linux distributions (may have limited support)
- Windows 10/11 (via WSL2 - limited)
- macOS (experimental support)

**Hardware:**
- 4+ CPU cores (Intel/AMD)
- 8 GB RAM minimum (16 GB recommended)
- 20 GB free disk space
- Internet connection for package downloads

**Software:**
- Python 3.8 or higher
- Git for version control
- Text editor or IDE

## Installation Methods

You have three primary installation options:

| Method | Pros | Cons | Best For |
|--------|------|------|----------|
| **Debian Packages** | Easy, official support | Ubuntu only | Most users |
| **Docker** | Isolated, portable | Overhead, complexity | Multi-platform |
| **Build from Source** | Latest features, customizable | Time-consuming, complex | Advanced users |

We'll cover **Debian packages** (recommended) and **Docker** methods.

## Method 1: Installing ROS 2 Humble (Debian Packages)

### Step 1: Set Locale

Ensure UTF-8 locale is configured:

```bash
# Check current locale
locale

# Set UTF-8 if needed
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

### Step 2: Setup Sources

Add the ROS 2 apt repository:

```bash
# Ensure Ubuntu Universe repository is enabled
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Packages

```bash
# Update package cache
sudo apt update

# Upgrade existing packages
sudo apt upgrade

# Install ROS 2 Humble Desktop (recommended for learning)
sudo apt install ros-humble-desktop

# Alternative: ROS 2 Base (minimal, no GUI tools)
# sudo apt install ros-humble-ros-base

# Install development tools
sudo apt install ros-dev-tools
```

**What's Included:**
- **Desktop**: ROS 2, RViz, demos, tutorials, rqt tools
- **Base**: Core ROS 2, no GUI tools (for servers/embedded)

### Step 4: Environment Setup

Source the ROS 2 setup file:

```bash
# Source ROS 2 (temporary - only for current terminal)
source /opt/ros/humble/setup.bash

# Make it permanent - add to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --help
```

You should see the ROS 2 command-line help output.

### Step 5: Install colcon (Build Tool)

```bash
# Install colcon (if not already installed with ros-dev-tools)
sudo apt install python3-colcon-common-extensions

# Verify installation
colcon --help
```

### Step 6: Test Installation

Run demo nodes to verify everything works:

```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal): Run listener
ros2 run demo_nodes_py listener
```

You should see the talker publishing messages and the listener receiving them.

## Method 2: Docker Installation

Docker provides an isolated ROS 2 environment without modifying your system.

### Install Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group (avoid sudo)
sudo usermod -aG docker $USER

# Logout and login for group changes to take effect
# Or run: newgrp docker

# Verify
docker run hello-world
```

### Get ROS 2 Docker Image

```bash
# Pull official ROS 2 Humble image
docker pull osrf/ros:humble-desktop

# Run container with GUI support (for rviz, rqt)
docker run -it \
  --name ros2_humble \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  osrf/ros:humble-desktop \
  bash

# Inside container, test ROS 2
ros2 run demo_nodes_cpp talker
```

**Docker Tips:**
- Save your work by committing container changes
- Use Docker volumes to persist workspace data
- Consider docker-compose for complex setups

### Docker Compose (Recommended)

Create `docker-compose.yml`:

```yaml
version: '3'
services:
  ros2:
    image: osrf/ros:humble-desktop
    container_name: ros2_dev
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./workspace:/workspace
    network_mode: host
    stdin_open: true
    tty: true
    command: bash
```

Run with:
```bash
docker-compose up -d
docker exec -it ros2_dev bash
```

## Creating Your First ROS 2 Workspace

A workspace is a directory where you modify, build, and install ROS 2 packages.

### Standard Workspace Structure

```
ros2_ws/                 # Workspace root
├── src/                 # Source space (your packages)
│   ├── package_1/
│   ├── package_2/
│   └── package_3/
├── build/               # Build artifacts (generated)
├── install/             # Installed packages (generated)
└── log/                 # Build logs (generated)
```

### Step-by-Step Workspace Setup

```bash
# 1. Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 2. Source ROS 2 installation
source /opt/ros/humble/setup.bash

# 3. Build workspace (even though empty)
colcon build

# Output:
# Starting >>> [empty]
# Finished <<< [empty] [0.15s]
```

### 4. Source the Workspace

```bash
# Source the workspace overlay
source ~/ros2_ws/install/setup.bash

# Make it permanent in .bashrc (optional - can conflict)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Better approach: Use alias
echo "alias ros2_ws='source ~/ros2_ws/install/setup.bash'" >> ~/.bashrc
```

**Overlay Concept:**
```
System Install (/opt/ros/humble)
       ↓
Workspace Overlay (~/ros2_ws/install)
       ↓
Your Packages Found First
```

## Essential ROS 2 Commands

Master these commands for daily development:

### Node Management

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /node_name

# Run a node
ros2 run <package_name> <executable_name>

# Example:
ros2 run turtlesim turtlesim_node
```

### Topic Management

```bash
# List all topics
ros2 topic list

# Echo (print) topic messages
ros2 topic echo /topic_name

# Get topic info (publishers, subscribers, type)
ros2 topic info /topic_name

# Publish to topic manually
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Check publishing frequency
ros2 topic hz /topic_name
```

### Service Management

```bash
# List services
ros2 service list

# Get service type
ros2 service type /service_name

# Call a service
ros2 service call /service_name service_type "{field: value}"

# Example:
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0}"
```

### Parameter Management

```bash
# List parameters for a node
ros2 param list /node_name

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name value

# Dump all parameters to file
ros2 param dump /node_name > params.yaml

# Load parameters from file
ros2 param load /node_name params.yaml
```

### Interface (Message/Service) Commands

```bash
# List all message types
ros2 interface list

# Show message definition
ros2 interface show std_msgs/msg/String

# Show service definition
ros2 interface show example_interfaces/srv/AddTwoInts

# Show action definition
ros2 interface show example_interfaces/action/Fibonacci
```

### Build and Development

```bash
# Build entire workspace
colcon build

# Build specific package
colcon build --packages-select package_name

# Build with output on screen (verbose)
colcon build --packages-select package_name --cmake-args ' -DCMAKE_BUILD_TYPE=Release'

# Clean build
rm -rf build install log
colcon build

# Build and run tests
colcon test

# Show test results
colcon test-result --all
```

## IDE Setup: Visual Studio Code

VS Code is the recommended IDE for ROS 2 development.

### Install VS Code

```bash
# Download and install
sudo apt install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] \
  https://packages.microsoft.com/repos/code stable main" | \
  sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
sudo apt update
sudo apt install code
```

### Essential VS Code Extensions

Install these extensions (Ctrl+Shift+X):

1. **ROS** (Microsoft)
   - Syntax highlighting for .msg, .srv, .action, .launch
   - Code completion
   - ROS commands integration

2. **Python** (Microsoft)
   - Python language support
   - IntelliSense, linting, debugging

3. **C/C++** (Microsoft)
   - C++ language support
   - IntelliSense for C++

4. **CMake** (twxs)
   - CMakeLists.txt syntax highlighting

5. **XML** (Red Hat)
   - XML formatting (for package.xml, launch files)

### Configure VS Code for ROS 2

Create `.vscode/settings.json` in your workspace:

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "${workspaceFolder}/install/lib/python3.10/site-packages"
  ],
  "python.analysis.extraPaths": [
    "/opt/ros/humble/lib/python3.10/site-packages",
    "${workspaceFolder}/install/lib/python3.10/site-packages"
  ],
  "cmake.configureOnOpen": false,
  "C_Cpp.default.includePath": [
    "/opt/ros/humble/include/**",
    "${workspaceFolder}/install/include/**"
  ]
}
```

### VS Code Integrated Terminal

Configure terminal to auto-source ROS 2:

```json
// In settings.json
"terminal.integrated.env.linux": {
  "ROS_DOMAIN_ID": "0"
},
"terminal.integrated.shellArgs.linux": [
  "-c",
  "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && bash"
]
```

## Environment Variables

Important ROS 2 environment variables:

| Variable | Purpose | Example |
|----------|---------|---------|
| `ROS_DOMAIN_ID` | Isolate ROS systems on same network | `export ROS_DOMAIN_ID=42` |
| `ROS_LOCALHOST_ONLY` | Restrict to local machine | `export ROS_LOCALHOST_ONLY=1` |
| `ROS_DISTRO` | ROS distribution | `humble` (auto-set) |
| `AMENT_PREFIX_PATH` | Installation prefixes | Auto-set by sourcing |
| `COLCON_LOG_LEVEL` | Build log verbosity | `export COLCON_LOG_LEVEL=debug` |

### ROS_DOMAIN_ID

Critical for multi-user or multi-robot scenarios:

```bash
# Set domain ID (0-101 for DDS, 0-232 for others)
export ROS_DOMAIN_ID=0

# Different users/robots should use different IDs
# User 1: export ROS_DOMAIN_ID=1
# User 2: export ROS_DOMAIN_ID=2
```

### ROS_LOCALHOST_ONLY

Prevent ROS traffic from going on network:

```bash
# Only communicate on localhost
export ROS_LOCALHOST_ONLY=1

# Allow network communication (default)
export ROS_LOCALHOST_ONLY=0
```

## Troubleshooting Common Issues

### Issue 1: "command not found: ros2"

**Cause:** ROS 2 not sourced  
**Fix:**
```bash
source /opt/ros/humble/setup.bash
```

### Issue 2: Package not found after building

**Cause:** Workspace not sourced  
**Fix:**
```bash
source ~/ros2_ws/install/setup.bash
```

### Issue 3: colcon build fails with "package not found"

**Cause:** Missing dependencies  
**Fix:**
```bash
# Install dependencies listed in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Issue 4: Nodes can't find each other

**Cause:** Different ROS_DOMAIN_ID or firewall  
**Fix:**
```bash
# Check domain ID matches
echo $ROS_DOMAIN_ID

# Check for firewall blocking
sudo ufw status

# Disable firewall temporarily to test
sudo ufw disable
```

### Issue 5: Permission denied errors

**Cause:** File ownership issues  
**Fix:**
```bash
# Fix ownership
sudo chown -R $USER:$USER ~/ros2_ws
```

### Issue 6: "Setup file not found"

**Cause:** Workspace not built yet  
**Fix:**
```bash
cd ~/ros2_ws
col build
```

## Best Practices

### 1. Workspace Organization

```
~/ros2_ws/
├── src/
│   ├── my_robot/              # Robot-specific packages
│   │   ├── my_robot_description/
│   │   ├── my_robot_bringup/
│   │   └── my_robot_control/
│   ├── common/                # Shared utilities
│   └── external/              # Third-party packages
├── build/
├── install/
└── log/
```

### 2. Use .gitignore

Create `.gitignore` in workspace root:

```
build/
install/
log/
*.pyc
__pycache__/
.vscode/
*.swp
```

### 3. Use colcon_cd

Navigate to package directories quickly:

```bash
# Add to .bashrc
source /usr/share/colcon_cd/function/colcon_cd.sh

# Usage
colcon_cd package_name
```

### 4. Faster Builds

```bash
# Use multiple cores
colcon build --parallel-workers 4

# Only build changed packages
colcon build --packages-up-to package_name

# Skip tests (faster)
colcon build --cmake-args -DBUILD_TESTING=OFF
```

## Summary

You now have a complete ROS 2 development environment with:
- ✓ ROS 2 Humble installed
- ✓ Workspace configured
- ✓ Essential tools (colcon, ros2 CLI)
- ✓ IDE setup (VS Code)
- ✓ Troubleshooting knowledge

## Key Takeaways

✓ Ubuntu 22.04 + ROS 2 Humble is the recommended setup  
✓ Always source your workspace before development  
✓ Use ROS_DOMAIN_ID for isolated development environments  
✓ VS Code with ROS extensions provides excellent developer experience  
✓ Understand the workspace overlay concept  

## Practice Exercise

1. Set up your ROS 2 environment following this guide
2. Create a workspace
3. Run the turtlesim demo:
```bash
ros2 run turtlesim turtlesim_node
```
4. In another terminal, control the turtle:
```bash
ros2 run turtlesim turtle_teleop_key
```
5. Explore topics the turtle publishes to:
```bash
ros2 topic list
ros2 topic echo /turtle1/pose
```

---

**Next:** Continue to [Packages and Nodes](./ros2-packages-nodes.md) to learn how to create your own ROS 2 packages.