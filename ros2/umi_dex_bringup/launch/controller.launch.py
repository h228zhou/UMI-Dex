"""Launch the CAN raw frame publisher node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "can_channel", default_value="can0",
            description="SocketCAN interface name",
        ),
        Node(
            package="umi_dex_bringup",
            executable="can_raw_node",
            name="can_raw",
            output="screen",
            parameters=[{
                "can_channel": LaunchConfiguration("can_channel"),
            }],
        ),
    ])
