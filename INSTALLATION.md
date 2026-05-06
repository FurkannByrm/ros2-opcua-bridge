# Magician ROS 2 OPC UA Control System — Installation Guide

This guide describes how to set up the full workspace on Ubuntu 22.04 with ROS 2 Humble.

It covers:

- `backend`
- `gui_app`
- `demonstrator_tree`
- the `open62541` system dependency

---

## 1. Requirements

| Component | Required version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| C++ | C++17 |
| Build system | `colcon`, `cmake` |
| OPC UA library | `open62541` >= 1.4 |
| Robot middleware | XBot2 (installed at `/opt/xbot`) |

Recommended minimum hardware:

- 4 GB RAM
- 10 GB free disk space

---

## 2. Prepare Ubuntu

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales curl software-properties-common git cmake build-essential
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

---

## 3. Install ROS 2 Humble

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep
```

Initialize `rosdep`:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Add ROS 2 to your shell:

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc \
  || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

Verify:

```bash
ros2 --version
```

---

## 4. Install system dependencies

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

The `demonstrator_tree` package subscribes to `xbot_msgs/msg/JointState` topics published by XBot2 robot controllers. XBot2 must be installed and its environment sourced before building.

XBot2 is installed system-wide under `/opt/xbot`. Headers used by this project live at:

```text
/opt/xbot/include/xbot_msgs/xbot_msgs/msg/
  joint_state.hpp   ← used by demonstrator_tree
  joint_command.hpp
  fault.hpp
  ...
```

Source the XBot2 environment before building or running:

```bash
source /opt/xbot/setup.sh
```

Add it permanently to your shell:

```bash
grep -qxF 'source /opt/xbot/setup.sh' ~/.bashrc \
  || echo 'source /opt/xbot/setup.sh' >> ~/.bashrc
```

> **Note — message type adaptability:** `demonstrator_tree` currently uses `xbot_msgs::msg::JointState` to read robot joint positions. If your robot stack does not use XBot2, you can replace this dependency with any other joint-state message type (e.g., `sensor_msgs/msg/JointState` from standard ROS 2). The only files that need to change are `include/demonstrator_tree/behavior_node.hpp` and `src/demonstrator_tree/behavior_node.cpp` — update the include, the subscription type, and the field name used to read link positions. The `CMakeLists.txt` and `package.xml` dependency entries for `xbot_msgs` must be updated to match the new package.

---

## 5. Install `open62541`

The `backend` package uses `find_package(open62541 REQUIRED)`, so `open62541` must be installed system-wide.

### 5.1 Clone

```bash
cd /tmp
git clone --depth 1 --branch v1.4.6 https://github.com/open62541/open62541.git
cd open62541
```

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

### 5.4 Verify

```bash
ls /usr/local/lib/cmake/open62541/open62541Config.cmake
ldconfig -p | grep open62541
pkg-config --modversion open62541
```

> Important: keep `UA_ENABLE_PUBSUB=OFF`. This project expects the same working configuration used by the test server and bridge.

---

## 6. Get the workspace

```bash
mkdir -p ~/magician_ws/src
cd ~/magician_ws/src
git clone https://github.com/FurkannByrm/ros2-opcua-bridge.git .
```

Expected structure:

```text
~/magician_ws/src/
├── backend/
├── demonstrator_tree/
├── gui_app/
├── INSTALLATION.md
└── README.md
```

---

## 7. Build the workspace

Source both ROS 2 and XBot2 before building:

```bash
cd ~/magician_ws
source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh
colcon build
source install/setup.bash
```

Optional: add the workspace overlay to your shell:

```bash
grep -qxF 'source ~/magician_ws/install/setup.bash' ~/.bashrc \
  || echo 'source ~/magician_ws/install/setup.bash' >> ~/.bashrc
```

Verify executables:

```bash
ros2 pkg executables backend
ros2 pkg executables gui_app
ros2 pkg executables demonstrator_tree
```

Expected binaries:

- `backend opc_bridge`
- `backend test_server`
- `gui_app gui_node`
- `demonstrator_tree demo`

---

## 8. Configuration

### 8.1 Backend OPC UA config

Production config:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3
```

Test config:

```yaml
endpoint: "opc.tcp://localhost:4840"
namespace_index: 3
```

Files:

- `backend/config/opcua.yaml`
- `backend/config/opcua_test.yaml`

### 8.2 BehaviorTree config

The current `demonstrator_tree/config/parameters.yaml` is configured for XBot2 interfaces:

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

The topic names (`/sr/xbotcore/joint_states`, `/cr/xbotcore/joint_states`) and service names (`/sr/xbotcore/homing/switch`, `/cr/xbotcore/homing/switch`) are the default XBot2 interfaces. If your XBot2 instance uses different namespaces, or if you are using a different robot middleware entirely, update `parameters.yaml` accordingly.

> Note: the `demo` executable currently loads its XML and YAML using workspace absolute paths, so keep the repository at the expected workspace location or update the source accordingly.

---

## 9. Run the system

### 9.1 Test mode

Recommended:

```bash
ros2 launch backend test_system.launch.py
```

Full stack with GUI:

```bash
ros2 launch backend full_test_system.launch.py
```

Manual mode:

```bash
ros2 run backend test_server
ros2 run backend opc_bridge --ros-args -p config:=opcua_test.yaml
ros2 run gui_app gui_node
```

### 9.2 Production mode

Edit `backend/config/opcua.yaml` with your PLC IP and then run:

```bash
ros2 launch backend system.launch.py
```

Or manually:

```bash
ros2 run backend opc_bridge
ros2 run gui_app gui_node
ros2 run demonstrator_tree demo
```

---

## 10. Verify runtime behavior

```bash
ros2 topic list | grep ros2_comm
ros2 service list | grep ros2_comm

ros2 topic echo /ros2_comm/speed
ros2 topic echo /ros2_comm/sensing/home_st

ros2 service call /ros2_comm/speed_set backend/srv/SetInt16 "{data: 500}"
ros2 service call /ros2_comm/mod/cobot_set std_srvs/srv/SetBool "{data: true}"
ros2 service call /ros2_comm/slider1/set_pos backend/srv/SetFloat32 "{data: 100.0}"
```

If `test_server` is running, OPC UA writes are printed to the terminal.

---

## 11. Troubleshooting

### Build issues

| Problem | Fix |
|---|---|
| `Could not find a package configuration file provided by "open62541"` | Reinstall `open62541` system-wide and run `sudo ldconfig` |
| `behaviortree_cpp` missing | Install `ros-humble-behaviortree-cpp` |
| `Qt5` missing | Install `qtbase5-dev` |
| `yaml-cpp` missing | Install `libyaml-cpp-dev` |

### Runtime issues

| Problem | Fix |
|---|---|
| Bridge cannot connect | Confirm PLC endpoint or start `test_server` |
| Namespace mismatch | Ensure `namespace_index` matches the server namespace |
| `Config file not found` | Rebuild and source `install/setup.bash` |
| Port `4840` busy | Stop the conflicting OPC UA server |

Check port usage:

```bash
lsof -i :4840
```

Clean rebuild:

```bash
cd ~/magician_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## 12. Quick start script

```bash
#!/bin/bash
set -e

echo "=== ROS 2 + dependencies ==="
sudo apt update && sudo apt upgrade -y
sudo apt install -y locales curl software-properties-common git cmake build-essential
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
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
git clone --depth 1 --branch v1.4.6 https://github.com/open62541/open62541.git
cd open62541
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
sudo make install
sudo ldconfig

echo "=== workspace ==="
mkdir -p ~/magician_ws/src
cd ~/magician_ws/src
git clone https://github.com/FurkannByrm/ros2-opcua-bridge.git .

echo "=== build ==="
cd ~/magician_ws
colcon build

grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qxF 'source ~/magician_ws/install/setup.bash' ~/.bashrc || echo 'source ~/magician_ws/install/setup.bash' >> ~/.bashrc
source install/setup.bash

echo "Done."
echo "Test mode: ros2 launch backend test_system.launch.py"
echo "Full test:  ros2 launch backend full_test_system.launch.py"
echo "Prod mode:  ros2 launch backend system.launch.py"
```
