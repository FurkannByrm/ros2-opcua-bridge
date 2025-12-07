
## System Overview

This workspace contains two main packages:

- **backend**: OPC-UA client that bridges PLC data to ROS2 topics and services
- **gui_app**: Qt5-based GUI application for monitoring and controlling the system

### Architecture

```
┌─────────────────┐      OPC-UA      ┌──────────────────┐      ROS2       ┌─────────────────┐
│   PLC/Server    │ ◄──────────────► │     backend      │ ◄──────────────► │    gui_app      │
│  (Test Server)  │    (open62541)   │  (ros_bridge)    │  (topics/srvs)  │  (Qt5 GUI)      │
└─────────────────┘                  └──────────────────┘                  └─────────────────┘
```

## Features

- **Real-time OPC-UA Communication**: Bidirectional data exchange with industrial PLCs
- **ROS2 Integration**: Full ROS2 pub/sub and service architecture
- **GUI Control Panel**: Modern Qt5 interface for system monitoring and control
- **Auto-reconnection**: Robust connection handling with exponential backoff
- **Multi-threaded**: Asynchronous command queue with mutex-protected operations
- **Configurable**: YAML-based configuration system

## Prerequisites

### System Requirements
- Ubuntu 22.04+ (ROS2 Foxy/Humble)
- ROS2 installed and sourced
- Qt5 development libraries
- CMake 3.8+

### Install Dependencies

```bash
# ROS2 dependencies
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-std-srvs

# Qt5 for GUI
sudo apt install qtbase5-dev qt5-qmake

# Build tools
sudo apt install build-essential cmake git
```

##  Building the Workspace

```bash
# Navigate to workspace
cd ~/ros_ws

# Build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Configuration

### Backend Configuration

Edit opcua.yaml:

```yaml
endpoint: "opc.tcp://localhost:4840"
namespace_index: 3

nodes:
  speed: "\"ROS2_COMM\".\"SPEED\""
  mode: "\"ROS2_COMM\".\"MODE\""
  command: "\"ROS2_COMM\".\"COMMAND\""

structs:
  mod_root: "\"ROS2_COMM\".\"MOD\""
  stat_root: "\"ROS2_COMM\".\"STAT\""
  sensing_root: "\"ROS2_COMM\".\"SENSING\""
  cleaning_root: "\"ROS2_COMM\".\"CLEANING\""

timing:
  sampling_ms: 100
  write_timeout_ms: 1000
  reconnect:
    initial_ms: 1000
    max_ms: 30000
    multiplier: 1.5
```

## Running the System

### Option 1: Complete System with Test Server

**Terminal 1** - Start the OPC-UA test server:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run backend test_server
```

**Terminal 2** - Start the backend bridge:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run backend main
```

**Terminal 3** - Launch the GUI:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run gui_app gui_node
```

### Option 2: Connect to Existing PLC

1. Update opcua.yaml with your PLC endpoint
2. Follow steps for Terminal 2 and 3 above

## ROS2 Topics & Services

### Published Topics (backend → gui_app)

| Topic | Type | Description |
|-------|------|-------------|
| `/ros2_comm/speed` | `std_msgs/Int16` | Current speed value |
| `/ros2_comm/mod/cobot` | `std_msgs/Bool` | COBOT mode state |
| `/ros2_comm/sensing/home_st` | `std_msgs/Bool` | Sensing robot home status |
| `/ros2_comm/sensing/finished` | `std_msgs/Bool` | Sensing operation finished |
| `/ros2_comm/sensing/sensing_active` | `std_msgs/Bool` | Sensing active state |
| `/ros2_comm/cleaning/home_st` | `std_msgs/Bool` | Cleaning robot home status |
| `/ros2_comm/cleaning/finished` | `std_msgs/Bool` | Cleaning operation finished |
| `/ros2_comm/cleaning/cleaning_active` | `std_msgs/Bool` | Cleaning active state |

### Services (gui_app → backend)

| Service | Type | Description |
|---------|------|-------------|
| `/ros2_comm/speed_set` | `backend/SetInt16` | Set speed value |
| `/ros2_comm/mod/cobot_set` | `std_srvs/SetBool` | Set COBOT mode |
| `/ros2_comm/sensing/safetransfer_set` | `std_srvs/SetBool` | Set sensing safe transfer |
| `/ros2_comm/sensing/finished_set` | `std_srvs/SetBool` | Set sensing finished |
| `/ros2_comm/sensing/active_set` | `std_srvs/SetBool` | Set sensing active |
| `/ros2_comm/cleaning/cobot_set` | `std_srvs/SetBool` | Set cleaning safe transfer |
| `/ros2_comm/cleaning/cleaning_finished_set` | `std_srvs/SetBool` | Set cleaning finished |
| `/ros2_comm/cleaning/cleaning_active_set` | `std_srvs/SetBool` | Set cleaning active |

##  GUI Features

The `mainwindow.cpp` provides:

- **Speed Control**: Set and monitor system speed (0-2000)
- **COBOT Mode**: Toggle collaborative robot mode
- **Sensing Robot Controls**: 7 toggle switches for sensing operations
- **Cleaning Robot Controls**: 5 toggle switches for cleaning operations
- **Real-time Updates**: Live feedback from PLC via ROS2 subscriptions
- **Status Bar**: Visual feedback for operations

### GUI Components

- Modern styled buttons with ON/OFF states (green/red)
- Automatic state synchronization with PLC
- Service call feedback
- Responsive scroll area for all controls

## 🔍 Key Implementation Details

### Backend Architecture

The `UaClient` implements:

1. **Asynchronous Command Queue**: Thread-safe write operations
2. **Subscription Management**: Shared subscriptions for efficiency
3. **Auto-reconnection**: Exponential backoff reconnection logic
4. **Worker Thread Pattern**: Separate thread for OPC-UA operations

```cpp
// Example: Write operation
ua_client->enqueue_write_bool(node_id, true);

// Example: Subscribe to changes
ua_client->subscribe_int16(node_id, [](int16_t value) {
    std::cout << "Value: " << value << std::endl;
});
```

### ROS Bridge

The `RosBridge` class:

- Converts OPC-UA subscriptions to ROS2 publishers
- Handles ROS2 service requests and writes to PLC
- Uses `make_child_node()` for node ID formatting

## Debugging

### Check ROS2 Communication

```bash
# List all topics
ros2 topic list

# Monitor a specific topic
ros2 topic echo /ros2_comm/speed

# Call a service manually
ros2 service call /ros2_comm/speed_set backend/srv/SetInt16 "{data: 1500}"
```

### Monitor OPC-UA Connection

The backend logs connection status:
```
[INFO] Connected successfully to OPC UA Server
[SUB] SPEED updated: 1200
[CMD] SPEED -> 1500
```

### GUI Debug

Check Qt console output for service call status:
```
Speed set to: 1500
Service not available!
```

## Code Structure

### Backend Package

```
backend/
├── include/backend/
│   ├── config.hpp           # YAML configuration loader
│   ├── opcua_client.hpp     # OPC-UA client interface
│   ├── ros_bridge.hpp       # ROS2 bridge class
│   └── naming.hpp           # Node ID utilities
├── src/
│   ├── config.cpp           # Config implementation
│   ├── opcua_client.cpp     # Client implementation
│   ├── ros_bridge.cpp       # Bridge implementation
│   └── main_with_ros.cpp    # Main entry point
├── test/
│   └── test_server.cpp      # OPC-UA test server
└── config/
    └── opcua.yaml           # Configuration file
```

### GUI Package

```
gui_app/
├── include/gui_app/
│   └── mainwindow.hpp       # Main window header
├── src/
│   ├── main.cpp             # Application entry
│   └── mainwindow.cpp       # GUI implementation
└── png/
    └── magician_logo_full.png  # Logo asset
```

Important Configuration Notes

The GUI application uses a hardcoded logo path that must be updated for your system:

 File: mainwindow.cpp (Line ~30)

