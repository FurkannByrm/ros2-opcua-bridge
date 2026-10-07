# Magician ROS 2 — Industrial Robot Cell Control System

An automation software system for managing an industrial robot cell (PLC, sensing/cleaning cobots, slider mechanisms) over ROS 2. It connects to the PLC via the OPC UA protocol, provides a Qt5-based operator interface, and includes BehaviorTree-based robot orchestration logic.

> For installation steps, see [INSTALLATION.md](INSTALLATION.md).

---

## Table of Contents

* [System Architecture](#system-architecture)
* [Packages](#packages)

  * [opcua_to_ros2 — OPC UA ↔ ROS 2 Bridge](#1-opcua_to_ros2)
  * [gui_app — Operator Interface](#2-gui_app)
  * [bt-ros2-bridge — BehaviorTree Orchestration](#3-bt-ros2-bridge)
* [Build and Run](#build-and-run)
* [ROS 2 Interfaces](#ros-2-interfaces)
* [OPC UA Address Space](#opc-ua-address-space)
* [Repository Layout](#repository-layout)
* [Debugging Tips](#debugging-tips)
* [Requirements](#requirements)
* [Maintainer](#maintainer)

---

## System Architecture

The system consists of three ROS 2 packages and follows a layered architecture:

```text
                         ┌──────────────────────────────────────────────────────────┐
                         │                    Operator Layer                        │
                         │                                                          │
                         │  ┌─────────────────────────────────────────────────────┐ │
                         │  │              gui_app  (Qt5 GUI)                     │ │
                         │  │  Speed control · Mode switching · Robot controls    │ │
                         │  │  Slider positions · Real-time status monitoring     │ │
                         │  └──────────────────────┬──────────────────────────────┘ │
                         │                         │ ROS 2 Service Calls            │
                         │                         │ ROS 2 Topic Subscriptions      │
                         └─────────────────────────┼────────────────────────────────┘
                                                   │
                         ┌─────────────────────────┼────────────────────────────────┐
                         │              Communication Layer                         │
                         │                         │                                │
                         │  ┌──────────────────────▼──────────────────────────────┐ │
                         │  │        opcua_to_ros2  (opc_bridge node)             │ │
                         │  │                                                     │ │
                         │  │  ┌─────────────┐   ┌────────────┐   ┌───────────┐   │ │
                         │  │  │ UaClient    │   │ RosBridge  │   │ Config    │   │ │
                         │  │  │ (open62541) │◄─►│ (rclcpp)   │   │ (YAML)    │   │ │
                         │  │  └──────┬──────┘   └────────────┘   └───────────┘   │ │
                         │  │         │                                           │ │
                         │  └─────────┼───────────────────────────────────────────┘ │
                         │            │ OPC UA (TCP)                                │
                         └────────────┼─────────────────────────────────────────────┘
                                      │
                         ┌────────────┼─────────────────────────────────────────────┐
                         │  Field     │  Layer                                      │
                         │            ▼                                             │
                         │  ┌──────────────────┐                                    │
                         │  │   Siemens PLC    │ ← Real production environment      │
                         │  └──────────────────┘                                    │
                         └──────────────────────────────────────────────────────────┘

                         ┌──────────────────────────────────────────────────────────┐
                         │               Orchestration Layer                        │
                         │                                                          │
                         │  ┌────────────────────────────────────────────────────┐  │
                         │  │         bt-ros2-bridge     (BehaviorTree.CPP)      │  │
                         │  │                                                    │  │
                         │  │  MainSequence                                      │  │
                         │  │  ├── WaitForSignal  → creation sub and client req  │  │
                         │  │  │                         as dinamicaly           │  │
                         │  │  └── ClientReq          → PLC operations           │  │
                         │  └────────────────────────────────────────────────────┘  │
                         │         │                                    │           │
                         │         │                                    │           │
                         │         │ /ros2_comm/sensing/running         │           │
                         │         │ expected_value:"true"              │           │
                         │         │ timeout="200.0"                    │           │
                         │         │                                    │           │
                         │         │ /xbotcore/homing/ergodic/homing    │           │
                         │         │ request_payload:"true"             │           │
                         │         │ timeout:"5.0"                      │           │
                         │         ▼                                    ▼           │
                         │  ┌──────────────┐                ┌──────────────────┐    │
                         │  │ Robot        │                │ opcua_to_ros2    │    │
                         │  │ Controllers  │                │ (opc_bridge)     │    │
                         │  │ (XBot2 /     │                │                  │    │
                         │  │  xbotcore)   │                │                  │    │
                         │  └──────────────┘                └──────────────────┘    │
                         └──────────────────────────────────────────────────────────┘
```

### Data Flow

```text
PLC --OPC UA subscription--> UaClient --> RosBridge --> ROS 2 topics --> GUI / BT

GUI --ROS 2 service--------> RosBridge --> UaClient --> OPC UA write --> PLC

BT  --ROS 2 service--------> RosBridge --> UaClient --> OPC UA write --> PLC
```

---

## Packages

### 1. `opcua_to_ros2`

The `opcua_to_ros2` package is the runtime communication core of the system.

It provides:

* OPC UA client connectivity through `open62541`
* Asynchronous write queue handling
* ROS 2 publishers for PLC state
* ROS 2 services for PLC commands
* Automatic reconnect handling
* YAML-based OPC UA configuration
* Production/test configuration switching through a ROS parameter
* ROS 2 launch files for production and test configurations

### Main Components

| Component      | Role                                                                                             |
| -------------- | ------------------------------------------------------------------------------------------------ |
| `UaClient`     | Handles OPC UA connection, subscriptions, reconnect logic, and queued writes on a worker thread. |
| `RosBridge`    | Exposes the ROS 2 API and forwards reads/writes between ROS 2 and OPC UA.                        |
| `ConfigLoader` | Loads bridge settings from YAML configuration files.                                             |

### Current Bridge Features

* `Int16` and `Bool` state bridging
* Slider target and slider actual-position handling with floating-point values
* Reconnect timing from YAML
* Production/test configuration switching through a ROS parameter
* ROS 2 launch files for production and test modes

### Configuration Files

Two bridge configuration files are shipped in:

```text
opcua_to_ros2/config/
```

| File              | Purpose                                            |
| ----------------- | -------------------------------------------------- |
| `opcua.yaml`      | Production configuration for the real PLC endpoint |
| `opcua_test.yaml` | Development/test configuration                     |

Example:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3

nodes:
  speed:   '"ROS2_COMM"."SPEED"'
  slider1: '"ROS2_COMM"."GO_TO_POS_1"'
  slider2: '"ROS2_COMM"."GO_TO_POS_2"'

structs:
  mod_root:      '"ROS2_COMM"."MOD"'
  stat_root:     '"ROS2_COMM"."STAT"'
  sensing_root:  '"ROS2_COMM"."STAT"."Robot_Sensing_Status"'
  cleaning_root: '"ROS2_COMM"."STAT"."Robot_Cleaning_Status"'
  Workcell:      '"ROS2_COMM"."MOD"."Workcell_Status"'

timing:
  sampling_ms: 50
  write_timeout_ms: 200
  reconnect: { initial_ms: 500, max_ms: 10000, multiplier: 2.0 }
```

---

### 2. `gui_app`

`gui_app` is a Qt5 Widgets-based operator interface.

It provides:

* Speed input
* Slider 1 / Slider 2 target position input
* Slider movement triggers
* Toggle controls for COBOT state
* Toggle controls for sensing robot signals
* Toggle controls for cleaning robot signals
* Live UI updates from ROS 2 subscriptions

### GUI Behavior

Each toggle button is connected to a ROS 2 `SetBool` service.

Incoming topic updates refresh the toggle state and styling.

Speed changes are sent through the custom:

```text
opcua_to_ros2/srv/SetInt16
```

service.

Slider position targets are sent through:

```text
opcua_to_ros2/srv/SetFloat32
```
## BehaviorTree Integration (`bt-ros2-interface`)

`bt-ros2-interface` coordinates the sequential execution of industrial operations (e.g., waiting for car body placement, sending slider move requests, and validating operation completion).

### XML Tree Definition

The behavior tree is defined in XML (`bt-ros2-interface/config/bt_tree.xml`) using BehaviorTree.CPP v4 format:

```

```

### Custom BT Nodes

1. **`WaitForSignal` (Condition / Sync Node):**

  * Subscribes to a boolean topic (`std_msgs/msg/Bool`).
  * Blocks execution until the topic publishes `expected_value` ("true"/"false") or until `timeout` (seconds) expires.
  * Returns `SUCCESS` when the condition is met, or `FAILURE` on timeout.
2. **`ClientReq` (Action Node):**

  * Triggers a ROS 2 command or service request on the specified `topic`.
  * Sends `request_payload="true"` and waits for acknowledgement within `timeout` seconds.
  * Returns `SUCCESS` on successful trigger or `FAILURE` on timeout/communication error.

---

## Installation &amp; Build Guide

### Prerequisites

Ensure you have Ubuntu 22.04 LTS and ROS 2 Humble installed.

```
# System Dependencies
sudo apt update
sudo apt install -y \
  ros-humble-behaviortree-cpp \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  libyaml-cpp-dev \
  qtbase5-dev

```

### 3.`bt-ros2-interface` &amp; Workspace Setup

1. **Create and prepare the ROS 2 workspace:**  
```  
mkdir -p ~/sample_ws/src  
cd ~/sample_ws/src  
```
2. **Clone the repositories:**  
```  
# Clone bt-ros2-interface  
git clone https://github.com/FurkannByrm/bt-ros2-interface.git  
# Clone ros2-opcua-bridge  
git clone https://github.com/FurkannByrm/ros2-opcua-bridge.git  
```
3. **Build the workspace:**  
```  
cd ~/sample_ws  
source /opt/ros/humble/setup.bash  
# Build bt-ros2-interface package individually:  
colcon build --packages-select bt-ros2-interface  
# Or build all packages together:  
colcon build  
# Source the overlay  
source install/setup.bash  
```

---

## How to Run

### Running `bt-ros2-interface`

#### Standalone Run

```
source ~/sample_ws/install/setup.bash
ros2 run bt-ros2-interface bt_ros2_node

```

#### Running with Custom Config / XML Parameters

```
ros2 run bt-ros2-interface bt_ros2_node --ros-args \
  -p bt_xml_filename:=bt_tree.xml \
  -p param_file:=parameters.yaml

```

#### Running via Launch File

```
ros2 launch bt-ros2-interface bt_launch.py

```

### Full System Execution

To run `bt-ros2-interface` alongside the OPC UA backend and GUI:

```
# Terminal 1: OPC UA Bridge Backend
ros2 launch backend system.launch.py

# Terminal 2: Operator GUI
ros2 run gui_app gui_node

# Terminal 3: BehaviorTree Orchestration Node
ros2 run bt-ros2-interface bt_ros2_node

```

### Testing &amp; Signal Simulation

You can manually trigger and test BT node signal transitions using ROS 2 CLI tools:

```
# Simulate carbody located signal (triggers step 1)
ros2 topic pub /ros2_comm/sensing/carbody_located_status std_msgs/msg/Bool "data: true" -1

# Simulate completion of operation 1
ros2 topic pub /is_operation1_finished std_msgs/msg/Bool "data: true" -1

# Simulate completion of operation 2
ros2 topic pub /is_operation2_finished std_msgs/msg/Bool "data: true" -1

# Monitor active slider commands issued by BT
ros2 topic echo /ros2_comm/sensing/go_slider_pos1

```

---

## ROS 2 Interfaces

### Key Topics Used by BehaviorTree

| Topic Name                                  | Type                | Description                              |
| ------------------------------------------- | ------------------- | ---------------------------------------- |
| `/ros2_comm/sensing/carbody_located_status` | `std_msgs/msg/Bool` | Signal indicating car body presence      |
| `/ros2_comm/sensing/go_slider_pos1..6`      | `std_msgs/msg/Bool` | Commands to send slider to positions 1-6 |
| `/is_operation1..4_finished`                | `std_msgs/msg/Bool` | Operation completion status signals      |
---

## ROS 2 Interfaces

The `opcua_to_ros2` bridge exposes the PLC state and commands through ROS 2 topics and services.

The interface is organized into three main areas:

* Common workcell and operation mode interfaces
* Sensing robot interfaces
* Cleaning robot interfaces
* Welding/spatter classification interface

---

### Published Topics

#### Common / Mode Topics

| Topic                      | Type                | Description                  |
| -------------------------- | ------------------- | ---------------------------- |
| `/ros2_comm/mod/cobot`     | `std_msgs/msg/Bool` | Current COBOT mode           |
| `/ros2_comm/mod/automatic` | `std_msgs/msg/Bool` | Current fully automatic mode |

The topics are populated from the corresponding OPC UA variables under:

```text
ROS2_COMM.MOD
├── COBOT
└── FULLY AUTOMATIC
```

---

### Sensing Topics

| Topic                                       | Type                   | Description                             |
| ------------------------------------------- | ---------------------- | --------------------------------------- |
| `/ros2_comm/sensing/home_st`                | `std_msgs/msg/Bool`    | Sensing robot safe-transfer/home status |
| `/ros2_comm/sensing/finished`               | `std_msgs/msg/Bool`    | Sensing operation finished              |
| `/ros2_comm/sensing/touch_finished`         | `std_msgs/msg/Bool`    | Touch sensing finished                  |
| `/ros2_comm/sensing/sensing_active`         | `std_msgs/msg/Bool`    | Sensing operation active                |
| `/ros2_comm/sensing/touch_active`           | `std_msgs/msg/Bool`    | Touch sensing active                    |
| `/ros2_comm/sensing/running`                | `std_msgs/msg/Bool`    | Sensing robot running                   |
| `/ros2_comm/sensing/carbody_located_status` | `std_msgs/msg/Bool`    | Car body successfully located           |
| `/ros2_comm/sensing/slider_actual_pos`      | `std_msgs/msg/Float32` | Actual sensing slider position          |
| `/ros2_comm/sensing/pos2_status`            | `std_msgs/msg/Bool`    | Sensing position 2 status               |
| `/ros2_comm/sensing/pos3_status`            | `std_msgs/msg/Bool`    | Sensing position 3 status               |
| `/ros2_comm/sensing/pos4_status`            | `std_msgs/msg/Bool`    | Sensing position 4 status               |
| `/ros2_comm/sensing/pos5_status`            | `std_msgs/msg/Bool`    | Sensing position 5 status               |
| `/ros2_comm/sensing/pos2_status_reached`    | `std_msgs/msg/Bool`    | Sensing position 2 enabled/reached      |
| `/ros2_comm/sensing/pos3_status_reached`    | `std_msgs/msg/Bool`    | Sensing position 3 enabled/reached      |
| `/ros2_comm/sensing/pos4_status_reached`    | `std_msgs/msg/Bool`    | Sensing position 4 enabled/reached      |
| `/ros2_comm/sensing/pos5_status_reached`    | `std_msgs/msg/Bool`    | Sensing position 5 enabled/reached      |

The corresponding OPC UA variables are located under:

```text
ROS2_COMM.STAT.Robot_Sensing_Status
```

Examples:

```text
robothome_safetransfer
sensing-finised
touchsensing-finished
sensing-active
touchsensing-active
running
Car_Poss_Ok
Sensing_Pos_2
Sensing_Pos_3
Sensing_Pos_4
Sensing_Pos_5
Sensing_Pos_2_Enable
Sensing_Pos_3_Enable
Sensing_Pos_4_Enable
Sensing_Pos_5_Enable
```

The actual slider position is read from:

```text
ROS2_COMM.MOD.Workcell_Status
└── Slider_1_actual position-linear
```

and published as:

```text
/ros2_comm/sensing/slider_actual_pos
```

---

### Cleaning Topics

| Topic                                        | Type                   | Description                              |
| -------------------------------------------- | ---------------------- | ---------------------------------------- |
| `/ros2_comm/cleaning/home_st`                | `std_msgs/msg/Bool`    | Cleaning robot safe-transfer/home status |
| `/ros2_comm/cleaning/finished`               | `std_msgs/msg/Bool`    | Cleaning operation finished              |
| `/ros2_comm/cleaning/cleaning_active`        | `std_msgs/msg/Bool`    | Cleaning operation active                |
| `/ros2_comm/cleaning/running`                | `std_msgs/msg/Bool`    | Cleaning robot running                   |
| `/ros2_comm/cleaning/carbody_located_status` | `std_msgs/msg/Bool`    | Car body successfully located            |
| `/ros2_comm/cleaning/slider_actual_pos`      | `std_msgs/msg/Float32` | Actual cleaning slider position          |
| `/ros2_comm/cleaning/pos2_status`            | `std_msgs/msg/Bool`    | Cleaning position 2 status               |
| `/ros2_comm/cleaning/pos3_status`            | `std_msgs/msg/Bool`    | Cleaning position 3 status               |
| `/ros2_comm/cleaning/pos4_status`            | `std_msgs/msg/Bool`    | Cleaning position 4 status               |
| `/ros2_comm/cleaning/pos5_status`            | `std_msgs/msg/Bool`    | Cleaning position 5 status               |
| `/ros2_comm/cleaning/pos2_status_reached`    | `std_msgs/msg/Bool`    | Cleaning position 2 enabled/reached      |
| `/ros2_comm/cleaning/pos3_status_reached`    | `std_msgs/msg/Bool`    | Cleaning position 3 enabled/reached      |
| `/ros2_comm/cleaning/pos4_status_reached`    | `std_msgs/msg/Bool`    | Cleaning position 4 enabled/reached      |
| `/ros2_comm/cleaning/pos5_status_reached`    | `std_msgs/msg/Bool`    | Cleaning position 5 enabled/reached      |

The corresponding OPC UA variables are located under:

```text
ROS2_COMM.STAT.Robot_Cleaning_Status
```

Examples:

```text
robothome_safetransfer
cleaning-finished
cleaning-active
running
Car_Pos_Ok
Cleaning_Pos_2
Cleaning_Pos_3
Cleaning_Pos_4
Cleaning_Pos_5
Cleaning_Pos_2_Enable
Cleaning_Pos_3_Enable
Cleaning_Pos_4_Enable
Cleaning_Pos_5_Enable
```

The actual slider position is read from:

```text
ROS2_COMM.MOD.Workcell_Status
└── Slider_2_actual position-linear
```

and published as:

```text
/ros2_comm/cleaning/slider_actual_pos
```

---

## ROS 2 Services

### Common / Mode Services

| Service                                  | Type                               | Description                                    |
| ---------------------------------------- | ---------------------------------- | ---------------------------------------------- |
| `/ros2_comm/mod/cobot_mode_set`          | `std_srvs/srv/SetBool`             | Enable/disable COBOT mode                      |
| `/ros2_comm/mod/full_automatic_mode_set` | `std_srvs/srv/SetBool`             | Enable/disable fully automatic mode            |
| `/ros2_comm/get_welding_spot_size`       | `opcua_to_ros2/srv/GetSpotWeights` | Retrieve detected welding spot spatter classes |

The mode services write directly to the corresponding OPC UA variables:

```text
ROS2_COMM.MOD.COBOT
ROS2_COMM.MOD.FULLY AUTOMATIC
```

For example:

```bash
ros2 service call /ros2_comm/mod/cobot_mode_set \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Enable fully automatic mode:

```bash
ros2 service call /ros2_comm/mod/full_automatic_mode_set \
  std_srvs/srv/SetBool \
  "{data: true}"
```

---

### Sensing Services

| Service                                 | Type                   | Description                            |
| --------------------------------------- | ---------------------- | -------------------------------------- |
| `/ros2_comm/sensing/safetransfer_set`   | `std_srvs/srv/SetBool` | Set sensing robot safe-transfer status |
| `/ros2_comm/sensing/finished_set`       | `std_srvs/srv/SetBool` | Set sensing finished                   |
| `/ros2_comm/sensing/touch_finished_set` | `std_srvs/srv/SetBool` | Set touch sensing finished             |
| `/ros2_comm/sensing/active_set`         | `std_srvs/srv/SetBool` | Set sensing active                     |
| `/ros2_comm/sensing/touch_active_set`   | `std_srvs/srv/SetBool` | Set touch sensing active               |
| `/ros2_comm/sensing/running`            | `std_srvs/srv/SetBool` | Set sensing running                    |
| `/ros2_comm/sensing/pos2_set`           | `std_srvs/srv/SetBool` | Set sensing position 2                 |
| `/ros2_comm/sensing/pos3_set`           | `std_srvs/srv/SetBool` | Set sensing position 3                 |
| `/ros2_comm/sensing/pos4_set`           | `std_srvs/srv/SetBool` | Set sensing position 4                 |
| `/ros2_comm/sensing/pos5_set`           | `std_srvs/srv/SetBool` | Set sensing position 5                 |

---

### Cleaning Services

| Service                                     | Type                   | Description                             |
| ------------------------------------------- | ---------------------- | --------------------------------------- |
| `/ros2_comm/cleaning/safetransfer_set`      | `std_srvs/srv/SetBool` | Set cleaning robot safe-transfer status |
| `/ros2_comm/cleaning/cleaning_finished_set` | `std_srvs/srv/SetBool` | Set cleaning finished                   |
| `/ros2_comm/cleaning/cleaning_active_set`   | `std_srvs/srv/SetBool` | Set cleaning active                     |
| `/ros2_comm/cleaning/running_set`           | `std_srvs/srv/SetBool` | Set cleaning running                    |
| `/ros2_comm/cleaning/pos2_set`              | `std_srvs/srv/SetBool` | Set cleaning position 2                 |
| `/ros2_comm/cleaning/pos3_set`              | `std_srvs/srv/SetBool` | Set cleaning position 3                 |
| `/ros2_comm/cleaning/pos4_set`              | `std_srvs/srv/SetBool` | Set cleaning position 4                 |
| `/ros2_comm/cleaning/pos5_set`              | `std_srvs/srv/SetBool` | Set cleaning position 5                 |

---

## Welding Integration

The welding integration provides the ROS 2 layer with the spatter classification information generated by the PLC/welding simulator.

The PLC exposes two spot groups:

```text
ROS2_COMM.STAT
├── Spot_M1
└── Spot_M2
```

The current bridge configuration contains twelve welding spot classification variables:

```yaml
spatter_classes:
  - '"Spot_1_Spatter_Class"'
  - '"Spot_2_Spatter_Class"'
  - '"Spot_3_Spatter_Class"'
  - '"Spot_4_Spatter_Class"'
  - '"Spot_5_Spatter_Class"'
  - '"Spot_6_Spatter_Class"'
  - '"Spot_7_Spatter_Class"'
  - '"Spot_8_Spatter_Class"'
  - '"Spot_9_Spatter_Class"'
  - '"Spot_10_Spatter_Class"'
  - '"Spot_11_Spatter_Class"'
  - '"Spot_12_Spatter_Class"'
```

Each welding spot can be classified into one of four spatter-size categories:

```text
Small
Small_Medium
Big_Medium
Big
```

### Welding Data Acquisition

At bridge initialization, `weldingLoad()` iterates over the configured spatter-size variables and creates an OPC UA subscription for each one.

Conceptually:

```text
PLC / OPC UA
     │
     │ subscription
     ▼
UaClient
     │
     ▼
weldingLoad()
     │
     ├── Small
     ├── Small_Medium
     ├── Big_Medium
     └── Big
          │
          ▼
   welding_lookup_
```

The bridge maintains:

```cpp
std::unordered_map<std::string, bool> welding_lookup_;
```

which stores the latest Boolean value for every spatter-size class.

The insertion order is stored separately in:

```cpp
std::vector<std::string> welding_insertion_order_;
```

This allows the response to preserve a deterministic order when the welding data is requested.

Because OPC UA callbacks may execute concurrently with a ROS 2 service callback, access to the welding data is protected by:

```cpp
std::mutex welding_mutex_;
```

The OPC UA callback updates the corresponding entry under the mutex:

```cpp
std::lock_guard<std::mutex> lock(welding_mutex_);
welding_lookup_[weld_topic] = v;
```

---

## Welding ROS 2 Service

The welding information is exposed through:

```text
/ros2_comm/get_welding_spot_size
```

with the custom service:

```text
opcua_to_ros2/srv/GetSpotWeights
```

The service response contains a list of:

```text
opcua_to_ros2/msg/Welding
```

messages.

The `Welding` message contains:

```text
welding_name
```

which identifies the active welding/spatter classification.

### Service Logic

When the service is called, the bridge:

1. Clears the previous response.
2. Locks the welding data mutex.
3. Iterates through the configured welding entries.
4. Checks the current Boolean value of each entry.
5. Adds an `opcua_to_ros2/msg/Welding` message to the response if the value is `true`.
6. Returns the list to the ROS 2 client.

Conceptually:

```text
ROS 2 client
     │
     │ /ros2_comm/get_welding_spot_size
     ▼
GetSpotWeights service
     │
     ▼
welding_lookup_
     │
     ├── Small          → false
     ├── Small_Medium   → true
     ├── Big_Medium     → false
     └── Big            → true
              │
              ▼
       Welding messages
              │
              ▼
          ROS 2 client
```

For example, if the current active classifications are:

```text
Small         = false
Small_Medium  = true
Big_Medium    = false
Big           = true
```

the service returns:

```text
weldings:
  - welding_name: "Small_Medium"
  - welding_name: "Big"
```

This approach keeps the OPC UA communication asynchronous while allowing other ROS 2 components to request the currently active welding classifications through a standard ROS 2 service call.

### Calling the Service

The service can be inspected with:

```bash
ros2 service type /ros2_comm/get_welding_spot_size
```

and called with:

```bash
ros2 service call /ros2_comm/get_welding_spot_size \
  opcua_to_ros2/srv/GetSpotWeights \
  "{}"
```

The service request is currently not used for filtering; the response is generated from the latest values stored by the OPC UA subscriptions.

---

## OPC UA Configuration

The current OPC UA configuration is:

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3

structs:
  mod_root: '"ROS2_COMM"."MOD"'
  Workcell: '"ROS2_COMM"."MOD"."Workcell_Status"'
  stat_root: '"ROS2_COMM"."STAT"'
  sensing_root: '"ROS2_COMM"."STAT"."Robot_Sensing_Status"'
  cleaning_root: '"ROS2_COMM"."STAT"."Robot_Cleaning_Status"'
  spot_M1_root: '"ROS2_COMM"."STAT"."Spot_M1"'
  spot_M2_root: '"ROS2_COMM"."STAT"."Spot_M2"'

  spatter_classes:
    - '"Spot_1_Spatter_Class"'
    - '"Spot_2_Spatter_Class"'
    - '"Spot_3_Spatter_Class"'
    - '"Spot_4_Spatter_Class"'
    - '"Spot_5_Spatter_Class"'
    - '"Spot_6_Spatter_Class"'
    - '"Spot_7_Spatter_Class"'
    - '"Spot_8_Spatter_Class"'
    - '"Spot_9_Spatter_Class"'
    - '"Spot_10_Spatter_Class"'
    - '"Spot_11_Spatter_Class"'
    - '"Spot_12_Spatter_Class"'

  spatter_size:
    - '"Small"'
    - '"Small_Medium"'
    - '"Big_Medium"'
    - '"Big"'

timing:
  sampling_ms: 50
  write_timeout_ms: 200
  reconnect:
    initial_ms: 500
    max_ms: 10000
    multiplier: 2.0
```

### OPC UA Structure

The relevant OPC UA address space is organized as:

```text
Objects/
└── ROS2_COMM
    ├── MOD/
    │   └── Workcell_Status/
    │       ├── Slider_1_actual position-linear
    │       └── Slider_2_actual position-linear
    │
    └── STAT/
        ├── Robot_Sensing_Status/
        │   ├── robothome_safetransfer
        │   ├── sensing-finised
        │   ├── touchsensing-finished
        │   ├── sensing-active
        │   ├── touchsensing-active
        │   ├── running
        │   ├── Car_Poss_Ok
        │   ├── Sensing_Pos_2
        │   ├── Sensing_Pos_3
        │   ├── Sensing_Pos_4
        │   ├── Sensing_Pos_5
        │   ├── Sensing_Pos_2_Enable
        │   ├── Sensing_Pos_3_Enable
        │   ├── Sensing_Pos_4_Enable
        │   └── Sensing_Pos_5_Enable
        │
        ├── Robot_Cleaning_Status/
        │   ├── robothome_safetransfer
        │   ├── cleaning-finished
        │   ├── cleaning-active
        │   ├── running
        │   ├── Car_Pos_Ok
        │   ├── Cleaning_Pos_2
        │   ├── Cleaning_Pos_3
        │   ├── Cleaning_Pos_4
        │   ├── Cleaning_Pos_5
        │   ├── Cleaning_Pos_2_Enable
        │   ├── Cleaning_Pos_3_Enable
        │   ├── Cleaning_Pos_4_Enable
        │   └── Cleaning_Pos_5_Enable
        │
        ├── Spot_M1/
        │   └── Welding / spatter classification data
        │
        └── Spot_M2/
            └── Welding / spatter classification data
```

---

## Data Flow

The overall communication flow is:

```text
                         ┌──────────────────────┐
                         │      Siemens PLC     │
                         │                      │
                         │      OPC UA Server   │
                         └──────────┬───────────┘
                                    │
                              OPC UA TCP
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │      UaClient        │
                         │    open62541         │
                         └──────────┬───────────┘
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │    OPCuaBridge       │
                         │                      │
                         │  OPC UA ↔ ROS 2      │
                         └───────┬───────┬──────┘
                                 │       │
                    subscriptions│       │services
                                 │       │
                  ┌──────────────┘       └──────────────┐
                  ▼                                     ▼
          ROS 2 Topics                            ROS 2 Services
                  │                                     │
          ┌───────┴────────┐                    ┌───────┴────────┐
          │                │                    │                │
          ▼                ▼                    ▼                ▼
        GUI              BT.CPP              GUI              BT.CPP
```

### Welding-specific flow

```text
PLC
 │
 │ OPC UA subscription
 ▼
Spot / Spatter classification
 │
 ▼
UaClient
 │
 ▼
welding_lookup_
 │
 │ ROS 2 service request
 ▼
/ros2_comm/get_welding_spot_size
 │
 ▼
GetSpotWeights response
 │
 ▼
opcua_to_ros2/msg/Welding[]
 │
 ▼
Planning / inspection / downstream ROS 2 component
```

The welding classification data is therefore **not continuously published as a ROS 2 topic**. Instead, the bridge continuously maintains the latest OPC UA values and exposes them on demand through the `GetSpotWeights` service.


---

## Build and Run

### Build

```bash
cd ~/magician_ws

source /opt/ros/humble/setup.bash
source /opt/xbot/setup.sh

colcon build

source install/setup.bash
```

### Production Mode

Edit:

```text
opcua_to_ros2/config/opcua.yaml
```

and set the real PLC endpoint.

Then run:

```bash
ros2 launch opcua_to_ros2 system.launch.py
```

Or run the packages manually:

```bash
ros2 run opcua_to_ros2 opc_bridge
ros2 run gui_app gui_node
```

---

## Repository Layout

```text
magician_ws/src/
├── opcua_to_ros2/
│   ├── config/
│   ├── include/opcua_to_ros2/
│   ├── launch/
│   ├── src/
│   ├── srv/
│   └── test/
│
├── gui_app/
│   ├── include/gui_app/
│   ├── png/
│   └── src/
│
├── INSTALLATION.md
└── README.md
```

---

## Debugging Tips

### ROS 2 Interfaces

```bash
ros2 topic list | grep ros2_comm
ros2 service list | grep ros2_comm
```

### Monitor Live Values

```bash
ros2 topic echo /ros2_comm/speed

ros2 topic echo /ros2_comm/sensing/home_st

ros2 topic echo /ros2_comm/cleaning/slider_actual_pos
```

### Manual Service Calls

Set speed:

```bash
ros2 service call /ros2_comm/speed_set \
  opcua_to_ros2/srv/SetInt16 \
  "{data: 500}"
```

Set COBOT mode:

```bash
ros2 service call /ros2_comm/mod/cobot_set \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Enable sensing:

```bash
ros2 service call /ros2_comm/sensing/active_set \
  std_srvs/srv/SetBool \
  "{data: true}"
```

Set slider 1 position:

```bash
ros2 service call /ros2_comm/slider1/set_pos \
  opcua_to_ros2/srv/SetFloat32 \
  "{data: 150.0}"
```

Trigger slider 1 movement:

```bash
ros2 service call /ros2_comm/slider1/go_pos \
  std_srvs/srv/SetBool \
  "{data: true}"
```

---

## Requirements

| Component        | Version                     |
| ---------------- | --------------------------- |
| Ubuntu           | 22.04 LTS                   |
| ROS 2            | Humble Hawksbill            |
| C++              | 17                          |
| Qt               | Qt5                         |
| OPC UA library   | open62541 >= 1.4            |
| BehaviorTree     | ros-humble-behaviortree-cpp |
| YAML             | libyaml-cpp-dev             |
| Robot middleware | XBot2 (`/opt/xbot`)         |

For full installation steps, see [INSTALLATION.md](INSTALLATION.md).

---

## Maintainer

**Furkan Bayram**

Email: `frknbyrm05@gmail.com`

