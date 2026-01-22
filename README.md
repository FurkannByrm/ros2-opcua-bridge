## System Overview

This workspace contains three main packages:

- **backend**: OPC-UA client that bridges PLC data to ROS2 topics and services
- **gui_app**: Qt5-based GUI application for monitoring and controlling the system
- **demonstrator_tree**: BehaviorTree-based orchestration system for coordinating robot operations

### Architecture

```
┌─────────────────┐      OPC-UA      ┌──────────────────┐      ROS2       ┌─────────────────┐
│   PLC/Server    │ ◄──────────────► │     backend      │ ◄──────────────► │    gui_app      │
│  (Test Server)  │    (open62541)   │  (ros_bridge)    │  (topics/srvs)  │  (Qt5 GUI)      │
└─────────────────┘                  └──────────────────┘                  └─────────────────┘
                                              ▲
                                              │ ROS2 Services
                                              │
                                     ┌────────┴──────────┐
                                     │ demonstrator_tree │
                                     │  (BehaviorTree)   │
                                     └───────────────────┘
                                              ▲
                                              │ ROS2 Services
                                              │
                                     ┌────────┴──────────┐
                                     │  Robot Controllers│
                                     │  (Sensing/Clean)  │
                                     └───────────────────┘
```

## Features

### Backend Package
- **Real-time OPC-UA Communication**: Bidirectional data exchange with industrial PLCs
- **ROS2 Integration**: Full ROS2 pub/sub and service architecture
- **Auto-reconnection**: Robust connection handling with exponential backoff
- **Multi-threaded**: Asynchronous command queue with mutex-protected operations
- **Configurable**: YAML-based configuration system

### GUI Application
- **Modern Qt5 Interface**: Real-time monitoring and control panel
- **Speed Control**: Set and monitor system speed (0-2000)
- **Robot Controls**: Toggle switches for sensing and cleaning operations
- **Live Feedback**: Real-time state synchronization with PLC

### Demonstrator Tree (NEW)
- **BehaviorTree Orchestration**: Coordinated robot operation sequences
- **Multi-Robot Coordination**: Simultaneous control of sensing and cleaning robots
- **Homing Management**: Automatic robot homing with position verification
- **Safe Transfer Protocol**: OPC-UA safe transfer signal management
- **Fault Tolerance**: Robust error handling and retry mechanisms

## Prerequisites

### System Requirements
- Ubuntu 22.04+ (ROS2 Jazzy/Humble)
- ROS2 installed and sourced
- Qt5 development libraries
- BehaviorTree.CPP library
- CMake 3.8+

### Install Dependencies

```bash
# ROS2 dependencies
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-std-msgs ros-${ROS_DISTRO}-std-srvs ros-${ROS_DISTRO}-sensor-msgs

# Qt5 for GUI
sudo apt install qtbase5-dev qt5-qmake

# BehaviorTree.CPP
sudo apt install ros-${ROS_DISTRO}-behaviortree-cpp

# YAML parser
sudo apt install libyaml-cpp-dev

# Build tools
sudo apt install build-essential cmake git
```

## Building the Workspace

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

Edit `backend/config/opcua.yaml`:

```yaml
endpoint: "opc.tcp://localhost:4840" // just sample
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

### Demonstrator Tree Configuration

Edit `demonstrator_tree/config/parameters.yaml`:

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

Edit `demonstrator_tree/config/bt_tree.xml`:

```xml
<root BTCPP_format="4">
 <BehaviorTree ID="MainTree">
    <Sequence name="magician_sequence">
     <Fallback>
        <IsRobotAtHome name="check_home_pos"/>
        <CallHoming name="call_homing_service"/>
     </Fallback>
     <CallOpcUI name="call_opcua_service"/>
    </Sequence>
 </BehaviorTree>
</root>
```

## Running the System

### Option 1: Complete System with Test Server

**Terminal 1** - Start the OPC-UA test server:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run backend test_server
```

**Terminal 2** - Start robot homing service simulator (for testing):
```bash
cd ~/ros_ws
source install/setup.bash
python3 src/demonstrator_tree/test/service_server.py
```

**Terminal 3** - Start the backend bridge:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run backend main
```

**Terminal 4** - Launch the GUI:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run gui_app gui_node
```

**Terminal 5** - Run the demonstrator tree:
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run demonstrator_tree demo
```

### Option 2: Connect to Real Robots

1. Update `opcua.yaml` with your PLC endpoint
2. Configure `parameters.yaml` with actual robot topics/services
3. Skip Terminal 2 (test server) and use real robot controllers

## Demonstrator Tree Package Details

### Purpose

The `demonstrator_tree` package implements a **BehaviorTree-based state machine** that orchestrates complex robot operations. It ensures robots are in safe positions before allowing system operations to proceed.

### Key Features

#### 1. **Multi-Robot Homing Coordination**
- Monitors joint states of two robots (sensing and cleaning)
- Verifies if robots are at home position (±0.01 rad tolerance)
- Triggers homing services if robots are not at home
- Supports parallel homing of both robots

#### 2. **BehaviorTree Control Flow**
The tree implements a **Fallback → Sequence** pattern:
- **Fallback**: Try to verify home position, if failed → call homing
- **Sequence**: After homing, enable OPC-UA safe transfer

#### 3. **Safe Transfer Protocol**
After successful homing:
- Signals to PLC via `/ros2_comm/sensing/safetransfer_set`
- Signals to PLC via `/ros2_comm/cleaning/safetransfer_set`
- Ensures system is ready for safe material transfer

### Architecture Components

#### **MagicianSubNode** (Subscriber Node)
- Subscribes to `/xbotcore/joint_states` for both robots
- Compares current joint positions with configured home positions
- Maintains static `robot_home_status_` map for state tracking
- Provides `checkPos()` for BehaviorTree condition checking

```cpp
// Example usage in BehaviorTree
factory.registerSimpleAction("IsRobotAtHome", [&](BT::TreeNode&){
    return subNode->checkPos();
});
```

#### **MagicianClientNode** (Service Client)
- Calls homing services: `/xbotcore/homing/switch1` and `switch2`
- Implements smart homing logic:
  - If only one robot needs homing → single call
  - If both need homing → parallel async calls
- 5-second timeout per service call
- Returns `BT::NodeStatus::SUCCESS/FAILURE`

```cpp
// Example: Parallel homing
auto sensing_future = sensing_client_->async_send_request(request);
auto cleaning_future = cleaning_client_->async_send_request(request);
```

#### **MagicianOpcUA** (OPC-UA Service Client)
- Calls backend safe transfer services
- Enables PLC signals after successful homing
- No response validation (fire-and-forget pattern)

### Initialization Sequence

1. **4-second warmup period**: Allows all subscribers to receive initial joint states
2. **MultiThreadedExecutor**: Spins all nodes simultaneously for async operations
3. **BehaviorTree registration**: Maps C++ functions to XML tree nodes
4. **Tree execution**: `tickWhileRunning()` runs until completion

### Error Handling

- **Timeout Protection**: All service calls have 5-second timeouts
- **State Verification**: Joint position tolerance of 0.01 radians
- **Failure Propagation**: BehaviorTree FAILURE stops execution
- **Logging**: Comprehensive RCLCPP_INFO/ERROR messages

### Use Case Example

**Scenario**: System startup sequence

1. User starts all ROS2 nodes
2. `demonstrator_tree` begins execution
3. **Check**: Are both robots at home?
   - **YES** → Skip to step 6
   - **NO** → Continue to step 4
4. **Homing**: Call homing service(s) for non-homed robot(s)
5. **Wait**: Services return success after robots reach home
6. **Safe Transfer**: Signal to PLC that robots are ready
7. **Complete**: Tree returns SUCCESS, system ready for operations

## ROS2 Topics & Services

### Backend Published Topics

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

### Backend Services

| Service | Type | Description |
|---------|------|-------------|
| `/ros2_comm/speed_set` | `backend/SetInt16` | Set speed value |
| `/ros2_comm/mod/cobot_set` | `std_srvs/SetBool` | Set COBOT mode |
| `/ros2_comm/sensing/safetransfer_set` | `std_srvs/SetBool` | Set sensing safe transfer |
| `/ros2_comm/sensing/finished_set` | `std_srvs/SetBool` | Set sensing finished |
| `/ros2_comm/sensing/active_set` | `std_srvs/SetBool` | Set sensing active |
| `/ros2_comm/cleaning/safetransfer_set` | `std_srvs/SetBool` | Set cleaning safe transfer |
| `/ros2_comm/cleaning/cleaning_finished_set` | `std_srvs/SetBool` | Set cleaning finished |
| `/ros2_comm/cleaning/cleaning_active_set` | `std_srvs/SetBool` | Set cleaning active |

### Robot Controller Services (Expected)

| Service | Type | Description |
|---------|------|-------------|
| `/xbotcore/homing/switch1` | `std_srvs/SetBool` | Sensing robot homing |
| `/xbotcore/homing/switch2` | `std_srvs/SetBool` | Cleaning robot homing |

### Robot Controller Topics (Expected)

| Topic | Type | Description |
|-------|------|-------------|
| `/xbotcore/joint_states` | `sensor_msgs/JointState` | Current joint positions |

## Debugging

### Check ROS2 Communication

```bash
# List all topics
ros2 topic list

# Monitor robot joint states
ros2 topic echo /xbotcore/joint_states

# Monitor OPC-UA speed
ros2 topic echo /ros2_comm/speed

# Call homing service manually
ros2 service call /xbotcore/homing/switch1 std_srvs/srv/SetBool "{data: true}"

# Call safe transfer service
ros2 service call /ros2_comm/sensing/safetransfer_set std_srvs/srv/SetBool "{data: true}"
```

### Monitor Demonstrator Tree

The tree logs detailed execution flow:
```
[INFO] sensing_cobot - ROBOT HOME POSITION
[INFO] cleaning_cobot - Robot NOT home
[INFO] CLEANING HOMING SUCCESS
[INFO] ALL COBOT HOMING SUCCESS
[INFO] Safe Transfer Home enabled.
```

### Test BehaviorTree Logic

Use the test server to simulate robot responses:
```bash
# Terminal 1: Start test homing server
python3 src/demonstrator_tree/test/service_server.py

# Terminal 2: Run demonstrator tree
ros2 run demonstrator_tree demo
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
├── srv/
│   └── SetInt16.srv         # Custom service definition
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

### Demonstrator Tree Package

```
demonstrator_tree/
├── include/demonstrator_tree/
│   ├── behavior_node.hpp        # BehaviorTree node classes
│   └── parameters_parser.hpp    # YAML config parser
├── src/
│   ├── main.cpp                 # Entry point + BT setup
│   ├── behavior_node.cpp        # Node implementations
│   └── parameters_parser.cpp    # Config loader
├── config/
│   ├── bt_tree.xml              # BehaviorTree structure
│   └── parameters.yaml          # Robot configuration
└── test/
    └── service_server.py        # Test homing server
```

## Important Configuration Notes

### Hardcoded Paths

The following files contain absolute paths that must be updated:

1. **GUI Logo Path** (`mainwindow.cpp` ~line 30)
2. **BehaviorTree XML** (`main.cpp` line with `createTreeFromFile`)
3. **Parameters YAML** (`main.cpp` line with `load_file`)

Update these paths to match your workspace location.

### Joint Tolerance

Homing verification uses 0.01 radian tolerance (defined in `behavior_node.hpp`):
```cpp
const double JOINT_TOL = 0.01;
```

Adjust if needed based on your robot's precision.

## Advanced Features

### Extending the BehaviorTree

Add new actions by registering them in `main.cpp`:

```cpp
factory.registerSimpleAction("MyAction", [&](BT::TreeNode&){
    // Your logic here
    return BT::NodeStatus::SUCCESS;
});
```

Then update `bt_tree.xml`:
```xml
<MyAction name="custom_action"/>
```

### Adding More Robots

1. Add robot config to `parameters.yaml`
2. Create new subscriber in `MagicianSubNode`
3. Create new service client in `MagicianClientNode`
4. Update `homingCall()` logic for N robots

## License

TODO: Add license information

## Maintainer

(frknbyrm05@gmail.com)