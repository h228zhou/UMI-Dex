"""Launch the controller raw publisher: CAN (SocketCAN) or USART (ttyUSB).

Select via `controller_protocol:=can|usart`.
Assembly/calibration are offline, so this node only publishes raw frames.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    protocol = LaunchConfiguration("controller_protocol")
    is_can = IfCondition(
        PythonExpression(["'", protocol, "' == 'can'"])
    )
    is_usart = IfCondition(
        PythonExpression(["'", protocol, "' == 'usart'"])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_protocol", default_value="can",
            description="Controller link: 'can' (SocketCAN) or 'usart' (ttyUSB)",
        ),
        DeclareLaunchArgument(
            "can_channel", default_value="can0",
            description="SocketCAN interface name",
        ),
        DeclareLaunchArgument(
            "usart_port", default_value="/dev/ttyUSB0",
            description="Serial device path for USART protocol",
        ),
        DeclareLaunchArgument(
            "usart_baud", default_value="115200",
            description="Serial baudrate for USART protocol",
        ),

        Node(
            package="umi_dex_bringup",
            executable="can_raw_node",
            name="can_raw",
            output="screen",
            condition=is_can,
            parameters=[{
                "can_channel": LaunchConfiguration("can_channel"),
            }],
        ),
        Node(
            package="umi_dex_bringup",
            executable="usart_raw_node",
            name="usart_raw",
            output="screen",
            condition=is_usart,
            parameters=[{
                "usart_port": LaunchConfiguration("usart_port"),
                "usart_baud": LaunchConfiguration("usart_baud"),
            }],
        ),
    ])
