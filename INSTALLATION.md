# Magician ROS 2 OPC-UA Bridge — Installation Guide

Step-by-step instructions for setting up the complete system (backend, gui_app,
demonstrator_tree) from scratch on a fresh Ubuntu 22.04 machine.

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Ubuntu 22.04 Setup](#2-ubuntu-2204-setup)
3. [ROS 2 Humble Installation](#3-ros-2-humble-installation)
4. [System Dependencies](#4-system-dependencies)
5. [open62541 — OPC UA Library](#5-open62541--opc-ua-library)
6. [Workspace Setup & Build](#6-workspace-setup--build)
7. [Running the System](#7-running-the-system)
8. [Configuration Reference](#8-configuration-reference)
9. [OPC UA Address Space](#9-opc-ua-address-space)
10. [Troubleshooting](#10-troubleshooting)
11. [Quick-Start Script](#11-quick-start-script)

---

## 1. System Requirements

| Component | Minimum |
|---|---|
| **OS** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **ROS 2** | Humble Hawksbill |
| **RAM** | 4 GB (8 GB recommended) |
| **Disk** | 10 GB free space |
| **Network** | Access to PLC (production) or localhost (test mode) |

### Software Dependencies

| Package | Purpose |
|---|---|
| `ros-humble-desktop` | Core ROS 2 packages (rclcpp, std_msgs, std_srvs, …) |
| `ros-humble-behaviortree-cpp` | BehaviorTree library (demonstrator_tree) |
| `libyaml-cpp-dev` | YAML config file parsing |
| `qtbase5-dev` | Qt5 GUI framework (gui_app) |
| **open62541** (≥ 1.4) | OPC UA C library — built from source |

---

## 2. Ubuntu 22.04 Setup

Install Ubuntu 22.04 LTS from the [official website](https://releases.ubuntu.com/22.04/).

After installation, update the system:

```bash
sudo apt update && sudo apt upgrade -y
```

---

## 3. ROS 2 Humble Installation

### 3.1 Locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3.2 Add the ROS 2 Repository

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

### 3.3 Install ROS 2 Humble Desktop

```bash
sudo apt install -y ros-humble-desktop
```

### 3.4 Build Tools

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake \
  git
```

### 3.5 Initialize rosdep

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

### 3.6 Source ROS 2 Automatically

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.7 Verify

```bash
ros2 --version
# Should print something like: ros2 0.x.x
```

---

## 4. System Dependencies

```bash
sudo apt install -y \
  libyaml-cpp-dev \
  qtbase5-dev \
  libqt5widgets5 \
  ros-humble-behaviortree-cpp \
  ros-humble-ament-index-cpp \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-sensor-msgs \
  ros-humble-launch \
  ros-humble-launch-ros
```

Verify:

```bash
dpkg -l | grep -cE "libyaml-cpp-dev|qtbase5-dev|ros-humble-behaviortree-cpp"
# Expected: 3
```

---

## 5. open62541 — OPC UA Library

The project depends on the [open62541](https://github.com/open62541/open62541)
OPC UA stack. It must be installed **system-wide** so that the workspace
`CMakeLists.txt` can locate it via `find_package(open62541)`.

### 5.1 Clone the Repository

```bash
cd /tmp
git clone --depth 1 --branch v1.4.6 https://github.com/open62541/open62541.git
cd open62541
```

> You can replace `v1.4.6` with any ≥ 1.4 release tag.
> List available tags with `git ls-remote --tags https://github.com/open62541/open62541.git | grep -oP 'v1\.\d+\.\d+$' | sort -V | tail`

### 5.2 Build

```bash
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DUA_ENABLE_DISCOVERY=ON \
  -DUA_ENABLE_PUBSUB=OFF \
  -DUA_ENABLE_PUBSUB_INFORMATIONMODEL=OFF \
  -DUA_NAMESPACE_ZERO=REDUCED \
  -DUA_ENABLE_AMALGAMATION=OFF

make -j$(nproc)
```

### 5.3 Install

```bash
sudo make install
sudo ldconfig
```

This places headers in `/usr/local/include/open62541/`, the shared library in
`/usr/local/lib/`, and CMake config files in `/usr/local/lib/cmake/open62541/`.

### 5.4 Verify

```bash
# CMake config exists
ls /usr/local/lib/cmake/open62541/open62541Config.cmake

# Library is loadable
ldconfig -p | grep open62541

# pkg-config works
pkg-config --modversion open62541
```

> **IMPORTANT:** `UA_ENABLE_PUBSUB` must be **OFF**. Enabling it with
> `UA_NAMESPACE_ZERO=REDUCED` causes a segfault because PubSub tries to
> register namespace-0 nodes (e.g. DataSetWriter i=15303) that do not exist in
> the reduced namespace.

### 5.5 Uninstall (if needed)

```bash
cd /tmp/open62541/build
sudo make uninstall    # or:  sudo xargs rm < install_manifest.txt
sudo ldconfig
```

---

## 6. Workspace Setup & Build

### 6.1 Create the Workspace

```bash
mkdir -p ~/magician_ws/src
cd ~/magician_ws/src
```

### 6.2 Clone the Source Code

```bash
git clone https://github.com/FurkannByrm/ros2-opcua-bridge.git .
```

> The trailing `.` clones directly into `src/`.

### 6.3 Verify Directory Structure

```
~/magician_ws/src/
├── backend/
├── demonstrator_tree/
├── gui_app/
└── README.md
```

### 6.4 Build All Packages

```bash
cd ~/magician_ws
source /opt/ros/humble/setup.bash
colcon build
```

> First build takes ~30 s. Subsequent builds are faster.

### 6.5 Source the Workspace

```bash
source ~/magician_ws/install/setup.bash
```

Make it permanent:

```bash
echo "source ~/magician_ws/install/setup.bash" >> ~/.bashrc
```

### 6.6 Verify

```bash
ros2 pkg executables backend
# backend opc_bridge
# backend test_server

ros2 pkg executables gui_app
# gui_app gui_node

ros2 pkg executables demonstrator_tree
# demonstrator_tree demo
```

### 6.7 Single-Package Build (during development)

```bash
colcon build --packages-select backend
colcon build --packages-select gui_app
colcon build --packages-select demonstrator_tree
```

---

## 7. Running the System

### 7.1 Test Mode (no PLC required)

The `test_server` executable simulates the PLC's OPC UA address space on
`localhost:4840`. All write operations are printed to the console in colour so
you can observe service calls in real time.

**Option A — Launch file (recommended)**

```bash
# Backend only (test_server + opc_bridge)
ros2 launch backend test_system.launch.py

# Full system (test_server + opc_bridge + GUI)
ros2 launch backend full_test_system.launch.py
```

**Option B — Manual (separate terminals)**

```bash
# Terminal 1 – Test server
ros2 run backend test_server

# Terminal 2 – Bridge (uses localhost config)
ros2 run backend opc_bridge --ros-args -p config:=opcua_test.yaml

# Terminal 3 – GUI (optional)
ros2 run gui_app gui_node

# Terminal 4 – Demonstrator tree (optional)
ros2 run demonstrator_tree demo
```

### 7.2 Production Mode (real PLC)

Edit `backend/config/opcua.yaml` and set the PLC endpoint:

```yaml
endpoint: "opc.tcp://<PLC_IP>:4840"
```

Then launch:

```bash
# Using the launch file (opc_bridge + GUI)
ros2 launch backend system.launch.py

# Or manually
ros2 run backend opc_bridge          # uses opcua.yaml by default
ros2 run gui_app gui_node
ros2 run demonstrator_tree demo
```

### 7.3 Verify the Running System

```bash
# List topics
ros2 topic list | grep ros2_comm

# Monitor speed
ros2 topic echo /ros2_comm/speed

# Call a service
ros2 service call /ros2_comm/speed_set backend/srv/SetInt16 "{data: 500}"
ros2 service call /ros2_comm/mod/cobot_set std_srvs/srv/SetBool "{data: true}"
```

---

## 8. Configuration Reference

Configuration files live in `backend/config/`.

| File | Purpose |
|---|---|
| `opcua.yaml` | Production — real PLC endpoint |
| `opcua_test.yaml` | Development — `opc.tcp://localhost:4840` |

Key parameters:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"   # PLC address
namespace_index: 3                         # OPC UA namespace

timing:
  sampling_ms: 50            # Subscription sampling interval
  write_timeout_ms: 200      # Write operation timeout
  reconnect:
    initial_ms: 500          # First reconnect delay
    max_ms: 10000            # Max reconnect delay
    multiplier: 2.0          # Exponential backoff factor
```

### Demonstrator Tree

`demonstrator_tree/config/parameters.yaml`:

```yaml
cobot1:
  robot_name: "sensing_cobot"
  sensing_joint_states: "/xbotcore/joint_states"
  sensing_service: "/xbotcore/homing/switch1"
  home_position: [0.0, 0.605, 1.571, 0.0, 0.995, 0.0]

cobot2:
  robot_name: "cleaning_cobot"
  cleaning_joint_states: "/xbotcore/joint_states"
  cleaning_service: "/xbotcore/homing/switch2"
  home_position: [0.0, 0.605, 1.571, 0.0, 0.995, 0.0]
```

---

## 9. OPC UA Address Space

Both the test server and the real PLC expose the same node tree:

```
Objects/
└── ROS2_COMM  (ns=3)
    ├── STATUS          Int16     System status
    ├── MODE            Int16     Operating mode
    ├── COMMAND         Int16     Command register
    ├── SPEED           Int16     Speed value (0-2000)
    ├── GO_TO_POS_1     Double    Slider 1 target position
    ├── GO_TO_POS_2     Double    Slider 2 target position
    ├── MOD/
    │   ├── STARTUP … SHUTDOWN_MODE   (8 × Bool)
    │   └── Workcell_Status/
    │       ├── Slider_1_actual position-linear   Double
    │       └── Slider_2_actual position-linear   Double
    └── STAT/
        ├── STARTUP … COBOT                       (6 × Bool)
        ├── Robot_Sensing_Status/
        │   ├── robothome_safetransfer             Bool
        │   ├── sensing-finised                    Bool
        │   ├── touchsensing-finished              Bool
        │   ├── sensing-active                     Bool
        │   ├── touchsensing-active                Bool
        │   ├── slide command                      Bool
        │   └── running                            Bool
        └── Robot_Cleaning_Status/
            ├── robothome_safetransfer             Bool
            ├── cleaning-finished                  Bool
            ├── cleaning-active                    Bool
            ├── slide command                      Bool
            └── running                            Bool
```

---

## 10. Troubleshooting

### Build Errors

| Error | Solution |
|---|---|
| `Could not find a package … 'open62541'` | Install open62541 system-wide (see [Section 5](#5-open62541--opc-ua-library)) |
| `Segmentation fault` (test_server) | Rebuild open62541 with `UA_ENABLE_PUBSUB=OFF` and reinstall |
| `behaviortree_cpp not found` | `sudo apt install ros-humble-behaviortree-cpp` |
| `Qt5 not found` | `sudo apt install qtbase5-dev` |
| `yaml-cpp not found` | `sudo apt install libyaml-cpp-dev` |

### Runtime Errors

| Error | Solution |
|---|---|
| `BadConnectionClosed` | PLC or test_server is not running |
| `Config file not found` | Run `source install/setup.bash` after building |
| Connection keeps dropping | Check the endpoint address in `opcua.yaml` |
| `Namespace mismatch` | Verify `namespace_index` matches the PLC |

### Port Conflict

The test server uses port 4840. If another OPC UA server is already running:

```bash
lsof -i :4840
kill <PID>
```

### Clean Rebuild

```bash
cd ~/magician_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 11. Quick-Start Script

A single script that performs every step on a fresh Ubuntu 22.04 system:

```bash
#!/bin/bash
set -e

echo "=== 1/6  ROS 2 Humble ==="
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions build-essential cmake git

echo "=== 2/6  Dependencies ==="
sudo apt install -y \
  libyaml-cpp-dev qtbase5-dev libqt5widgets5 \
  ros-humble-behaviortree-cpp \
  ros-humble-ament-index-cpp \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-sensor-msgs \
  ros-humble-launch ros-humble-launch-ros

echo "=== 3/6  open62541 ==="
source /opt/ros/humble/setup.bash
cd /tmp
git clone --depth 1 --branch v1.4.6 https://github.com/open62541/open62541.git
cd open62541 && mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DUA_ENABLE_DISCOVERY=ON \
  -DUA_ENABLE_PUBSUB=OFF \
  -DUA_ENABLE_PUBSUB_INFORMATIONMODEL=OFF \
  -DUA_NAMESPACE_ZERO=REDUCED \
  -DUA_ENABLE_AMALGAMATION=OFF
make -j$(nproc)
sudo make install
sudo ldconfig

echo "=== 4/6  Workspace ==="
mkdir -p ~/magician_ws/src && cd ~/magician_ws/src
git clone https://github.com/FurkannByrm/ros2-opcua-bridge.git .

echo "=== 5/6  Build ==="
cd ~/magician_ws
colcon build

echo "=== 6/6  Environment ==="
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qxF 'source ~/magician_ws/install/setup.bash' ~/.bashrc \
  || echo 'source ~/magician_ws/install/setup.bash' >> ~/.bashrc
source install/setup.bash

echo ""
echo "Installation complete!"
echo "  Test mode : ros2 launch backend test_system.launch.py"
echo "  Production: ros2 launch backend system.launch.py"
```
