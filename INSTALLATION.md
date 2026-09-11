# Magician ROS 2 OPC UA Control System — Installation Guide

This guide describes how to set up the full workspace on Ubuntu 22.04 with ROS 2 Humble.

The workspace contains the following packages:

* `opcua_to_ros2`
* `gui_app`
* `demonstrator_tree`
* the `open62541` system dependency

The `opcua_to_ros2` package provides the OPC UA ↔ ROS 2 communication bridge used to exchange data between the PLC/OPC UA system and ROS 2.

---

## 1. Requirements

| Component        | Required version                 |
| ---------------- | -------------------------------- |
| OS               | Ubuntu 22.04 LTS                 |
| ROS              | ROS 2 Humble                     |
| C++              | C++17                            |
| Build system     | `colcon`, `cmake`                |
| OPC UA library   | `open62541` >= 1.4               |
| Robot middleware | XBot2 (installed at `/opt/xbot`) |

Recommended minimum hardware:

* 4 GB RAM
* 10 GB free disk space

---

## 2. Prepare Ubuntu

Update the system and install the basic development tools:

```bash
sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  locales \
  curl \
  software-properties-common \
  git \
  cmake \
  build-essential
```

Configure the locale:

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

---

## 3. Install ROS 2 Humble

Add the ROS 2 repository:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update the package index:

```bash
sudo apt update
```

Install ROS 2 Humble and the required ROS development tools:

```bash
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep
```

### Initialize rosdep

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

### Source ROS 2

Add ROS 2 to `.bashrc`:

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

source /opt/ros/humble/setup.bash
```

Verify the installation:

```bash
ros2 --version
```

---

## 4. Install System Dependencies

Install the system and ROS 2 dependencies required by the workspace:

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
  ros-humble-launch-ros \
  pkg-config
```

---

## 4.1 Install XBot2

The `demonstrator_tree` package subscribes to `xbot_msgs/msg/JointState` topics published by XBot2 robot controllers.

XBot2 must be installed and its environment sourced before building or running the workspace.

XBot2 is installed system-wide under:

```text
/opt/xbot
```

Headers used by this project are located at:

```text
/opt/xbot/include/xbot_msgs/xbot_msgs/msg/
  joint_state.hpp
  joint_command.hpp
  fault.hpp
  ...
```

Source the XBot2 environment:

```bash
source /opt/xbot/setup.sh
```

To source it automatically for every terminal:

```bash
grep -qxF 'source /opt/xbot/setup.sh' ~/.bashrc \
  || echo 'source /opt/xbot/setup.sh' >> ~/.bashrc
```

### Note — message type adaptability

`demonstrator_tree` currently uses `xbot_msgs::msg::JointState` to read robot joint positions.

If your robot stack does not use XBot2, you can replace this dependency with another joint-state message type, such as:

```text
sensor_msgs/msg/JointState
```

The relevant source files are:

```text
include/demonstrator_tree/behavior_node.hpp
src/demonstrator_tree/behavior_node.cpp
```

If the message type is changed, update the corresponding dependencies in:

```text
demonstrator_tree/CMakeLists.txt
demonstrator_tree/package.xml
```

---

## 5. Install open62541

The `opcua_to_ros2` package uses:

```cmake
find_package(open62541 REQUIRED)
```

Therefore, `open62541` must be installed system-wide.

This project uses `open62541` version 1.4.6.

### 5.1 Clone

```bash
cd /tmp

git clone --depth 1 \
  --branch v1.4.6 \
  https://github.com/open62541/open62541.git

cd open62541
```

### 5.2 Build

Create the build directory:

```bash
mkdir build
cd build
```

Configure the project:

```bash
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DUA_ENABLE_DISCOVERY=ON \
  -DUA_ENABLE_PUBSUB=OFF \
  -DUA_ENABLE_PUBSUB_INFORMATIONMODEL=OFF \
  -DUA_NAMESPACE_ZERO=REDUCED \
  -DUA_ENABLE_AMALGAMATION=OFF
```

Build:

```bash
make -j$(nproc)
```

### 5.3 Install

```bash
sudo make install
sudo ldconfig
```

### 5.4 Verify

Check that the CMake configuration file exists:

```bash
ls /usr/local/lib/cmake/open62541/open62541Config.cmake
```

Check the installed library:

```bash
ldconfig -p | grep open62541
```

Check the installed version:

```bash
pkg-config --modversion open62541
```

> **Important:** Keep `UA_ENABLE_PUBSUB=OFF`. The ROS 2 OPC UA bridge does not require OPC UA PubSub functionality.

---

## 6. Get the Workspace

Create the ROS 2 workspace:

```bash
mkdir -p ~/magician_ws/src
cd ~/magician_ws/src
```

Clone the repository:

```bash
git clone https://github.com/FurkannByrm/ros2-opcua-bridge.git .
```

Expected workspace structure:

```text
~/magician_ws/src/
├── opcua_to_ros2/
├── demonstrator_tree/
├── gui_app/
├── INSTALLATION.md
└── README.md
```

The OPC UA bridge package is now named:

```text
opcua_to_ros2
```

---

## 7. Build the Workspace

Source ROS 2 and XBot2 before building:

```bash
cd ~/magician_ws

source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh
```

Build the workspace:

```bash
colcon build
```

After a successful build:

```bash
source install/setup.bash
```

### Optional: Source the workspace automatically

```bash
grep -qxF 'source ~/magician_ws/install/setup.bash' ~/.bashrc \
  || echo 'source ~/magician_ws/install/setup.bash' >> ~/.bashrc
```

---

## 7.1 Verify the Packages

Check the installed packages:

```bash
ros2 pkg list | grep -E 'opcua_to_ros2|gui_app|demonstrator_tree'
```

Check the executables:

```bash
ros2 pkg executables opcua_to_ros2
ros2 pkg executables gui_app
ros2 pkg executables demonstrator_tree
```

Expected executables:

```text
opcua_to_ros2 opc_bridge
gui_app gui_node
demonstrator_tree demo
```

---

## 8. Configuration

### 8.1 OPC UA Configuration

The OPC UA bridge uses the following configuration file:

```text
opcua_to_ros2/config/opcua.yaml
```

Example:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3
```

Update the endpoint according to the PLC/OPC UA server configuration.

For example:

```yaml
endpoint: "opc.tcp://<PLC_IP>:4840"
namespace_index: 3
```

> **Important:** The `namespace_index` must match the namespace used by the OPC UA server.

---

## 8.2 BehaviorTree Configuration

The current `demonstrator_tree/config/parameters.yaml` is configured for XBot2 interfaces.

Example:

```yaml
cobot1:
  robot_name: "sensing_cobot"
  sensing_joint_states: "/sr/xbotcore/joint_states"
  sensing_service: "/sr/xbotcore/homing/switch"
  home_position: [0.0036, 0.6, 1.57, 0.003, 0.99, 0.005]

cobot2:
  robot_name: "cleaning_cobot"
  cleaning_joint_states: "/cr/xbotcore/joint_states"
  cleaning_service: "/cr/xbotcore/homing/switch"
  home_position: [0.0036, 0.6, 1.57, 0.003, 0.99, 0.005]
```

The topic names:

```text
/sr/xbotcore/joint_states
/cr/xbotcore/joint_states
```

and service names:

```text
/sr/xbotcore/homing/switch
/cr/xbotcore/homing/switch
```

are the default XBot2 interfaces.

If your XBot2 configuration uses different namespaces, update:

```text
demonstrator_tree/config/parameters.yaml
```

accordingly.

> **Note:** The `demo` executable currently loads its XML and YAML configuration using workspace absolute paths. Keep the repository at the expected workspace location or update the source accordingly.

---

## 9. Run the System

### 9.1 Production Mode

First, configure the PLC endpoint in:

```text
opcua_to_ros2/config/opcua.yaml
```

For example:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3
```

Then source the required environments:

```bash
source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh
source ~/magician_ws/install/setup.bash
```

### Launch the complete system

```bash
ros2 launch opcua_to_ros2 system.launch.py
```

### Manual startup

The OPC UA bridge can also be started manually:

```bash
ros2 run opcua_to_ros2 opc_bridge
```

Start the GUI:

```bash
ros2 run gui_app gui_node
```

Start the BehaviorTree application:

```bash
ros2 run demonstrator_tree demo
```

---

## 10. Verify Runtime Behavior

Check the ROS 2 topics:

```bash
ros2 topic list | grep ros2_comm
```

Check the ROS 2 services:

```bash
ros2 service list | grep ros2_comm
```

For example:

```bash
ros2 topic echo /ros2_comm/speed
```

```bash
ros2 topic echo /ros2_comm/sensing/home_st
```

Example service calls:

```bash
ros2 service call \
  /ros2_comm/speed_set \
  opcua_to_ros2/srv/SetInt16 \
  "{data: 500}"
```

```bash
ros2 service call \
  /ros2_comm/mod/cobot_set \
  std_srvs/srv/SetBool \
  "{data: true}"
```

```bash
ros2 service call \
  /ros2_comm/slider1/set_pos \
  opcua_to_ros2/srv/SetFloat32 \
  "{data: 100.0}"
```

> The exact topics and services depend on the interfaces implemented by the current `opcua_to_ros2` package.

---

## 11. Troubleshooting

### Build Issues

| Problem                                                               | Fix                                                       |
| --------------------------------------------------------------------- | --------------------------------------------------------- |
| `Could not find a package configuration file provided by "open62541"` | Reinstall `open62541` system-wide and run `sudo ldconfig` |
| `behaviortree_cpp` missing                                            | Install `ros-humble-behaviortree-cpp`                     |
| `Qt5` missing                                                         | Install `qtbase5-dev`                                     |
| `yaml-cpp` missing                                                    | Install `libyaml-cpp-dev`                                 |

### Runtime Issues

| Problem                 | Fix                                                          |
| ----------------------- | ------------------------------------------------------------ |
| Bridge cannot connect   | Confirm the PLC/OPC UA endpoint and network connection       |
| Namespace mismatch      | Ensure `namespace_index` matches the OPC UA server namespace |
| `Config file not found` | Rebuild the workspace and source `install/setup.bash`        |
| Port `4840` busy        | Check whether another OPC UA server is using port `4840`     |

Check port usage:

```bash
lsof -i :4840
```

---

## 11.1 Clean Rebuild

If the workspace has build or package configuration issues, perform a clean rebuild:

```bash
cd ~/magician_ws

rm -rf build install log

source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh

colcon build

source install/setup.bash
```

---

## 12. Quick Start Script

The following script installs the required dependencies, builds `open62541`, clones the workspace, and builds the ROS 2 packages.

```bash
#!/bin/bash
set -e

echo "=== ROS 2 + dependencies ==="

sudo apt update && sudo apt upgrade -y

sudo apt install -y \
  locales \
  curl \
  software-properties-common \
  git \
  cmake \
  build-essential

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  libyaml-cpp-dev \
  qtbase5-dev \
  libqt5widgets5 \
  ros-humble-behaviortree-cpp \
  ros-humble-ament-index-cpp \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-sensor-msgs \
  ros-humble-launch \
  ros-humble-launch-ros \
  pkg-config

sudo rosdep init 2>/dev/null || true
rosdep update

echo "=== open62541 ==="

source /opt/ros/humble/setup.bash

cd /tmp

git clone \
  --depth 1 \
  --branch v1.4.6 \
  https://github.com/open62541/open62541.git

cd open62541

mkdir build
cd build

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

echo "=== workspace ==="

mkdir -p ~/magician_ws/src

cd ~/magician_ws/src

git clone \
  https://github.com/FurkannByrm/ros2-opcua-bridge.git .

echo "=== build ==="

cd ~/magician_ws

source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh

colcon build

grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

grep -qxF 'source /opt/xbot/setup.sh' ~/.bashrc \
  || echo 'source /opt/xbot/setup.sh' >> ~/.bashrc

grep -qxF 'source ~/magician_ws/install/setup.bash' ~/.bashrc \
  || echo 'source ~/magician_ws/install/setup.bash' >> ~/.bashrc

source install/setup.bash

echo ""
echo "========================================"
echo "Installation completed successfully."
echo "========================================"
echo ""
echo "Production system:"
echo "  ros2 launch opcua_to_ros2 system.launch.py"
echo ""
echo "Manual OPC UA bridge:"
echo "  ros2 run opcua_to_ros2 opc_bridge"
echo ""
echo "GUI:"
echo "  ros2 run gui_app gui_node"
echo ""
echo "BehaviorTree:"
echo "  ros2 run demonstrator_tree demo"
```

---

## 13. Package Overview

### `opcua_to_ros2`

Provides the OPC UA ↔ ROS 2 communication layer.

Main responsibilities include:

* OPC UA client communication
* Reading PLC/OPC UA values
* Writing ROS 2 commands to OPC UA
* Publishing OPC UA data as ROS 2 topics
* Exposing ROS 2 services for OPC UA write/read operations
* Handling the communication between the PLC layer and ROS 2

### `gui_app`

Provides the graphical user interface for interacting with the system.

### `demonstrator_tree`

Contains the BehaviorTree-based robot coordination and automation logic.

It interfaces with the robot middleware and ROS 2 components required by the demonstrator.

---

## 14. System Architecture

The overall communication flow is:

```text
                    ┌─────────────────────┐
                    │        PLC          │
                    │     OPC UA Server   │
                    └──────────┬──────────┘
                               │
                               │ OPC UA
                               │
                    ┌──────────▼──────────┐
                    │    opcua_to_ros2     │
                    │     opc_bridge       │
                    └──────────┬──────────┘
                               │
                               │ ROS 2
              ┌────────────────┼────────────────┐
              │                │                │
              ▼                ▼                ▼
       ┌────────────┐   ┌───────────────┐   ┌──────────────┐
       │  gui_app   │   │ demonstrator  │   │ Other ROS 2  │
       │            │   │     _tree     │   │    nodes     │
       └────────────┘   └───────────────┘   └──────────────┘
```

The `opcua_to_ros2` package acts as the communication bridge between the industrial PLC/OPC UA layer and the ROS 2 ecosystem.

---

## 15. Notes

* The system requires a working OPC UA server for runtime operation.
* The PLC endpoint must be reachable from the host machine.
* The configured OPC UA namespace index must match the server configuration.
* XBot2 must be sourced before starting `demonstrator_tree`.
* `open62541` must be installed system-wide before building `opcua_to_ros2`.
* After changing package names, dependencies, or interfaces, perform a clean build if necessary.
* Always source the ROS 2 workspace before running the system:

```bash
source ~/magician_ws/install/setup.bash
```

For a complete production startup:

```bash
source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh
source ~/magician_ws/install/setup.bash

ros2 launch opcua_to_ros2 system.launch.py
```

