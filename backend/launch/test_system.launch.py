"""
Launch file for running the system with the OPC UA test server.

Usage:
    ros2 launch backend test_system.launch.py

This starts:
  1. test_server  - OPC UA server simulating the real PLC (localhost:4840)
  2. opc_bridge   - ROS 2 <-> OPC UA bridge (using opcua_test.yaml)
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

    # Give the test server 2 seconds to initialize before starting the bridge
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

    return LaunchDescription([
        test_server,
        opc_bridge,
    ])
