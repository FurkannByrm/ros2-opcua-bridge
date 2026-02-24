"""
Launch file for the full Magician system with the test server.

Starts all three packages:
  1. test_server   - OPC UA server simulating the PLC
  2. opc_bridge    - ROS 2 <-> OPC UA bridge
  3. gui_node      - Qt5 GUI application

Usage:
    ros2 launch backend full_test_system.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    test_server = Node(
        package='backend',
        executable='test_server',
        name='opcua_test_server',
        output='screen',
    )

    opc_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='backend',
                executable='opc_bridge',
                name='opc_bridge',
                output='screen',
                parameters=[{'config': 'opcua_test.yaml'}],
            ),
        ],
    )

    gui_node = TimerAction(
        period=4.0,
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
        test_server,
        opc_bridge,
        gui_node,
    ])
