"""
Launch file for the production system (real PLC).

Usage:
    ros2 launch backend system.launch.py

Starts:
  1. opc_bridge  - ROS 2 <-> OPC UA bridge (using default opcua.yaml)
  2. gui_node    - Qt5 operator GUI (after 2 s delay)

Note: No test_server is started. The real PLC must be reachable at the
endpoint defined in backend/config/opcua.yaml.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    opc_bridge = Node(
        package='backend',
        executable='opc_bridge',
        name='opc_bridge',
        output='screen',
        # Uses the default opcua.yaml (real PLC endpoint)
    )

    gui_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gui_app',
                executable='gui_node',
                name='gui_node',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        opc_bridge,
        gui_node,
    ])
