"""Full capture pipeline: D455 + D405 + controller (CAN or USART) + interactive recorder.

Usage:
  ros2 launch umi_dex_bringup capture.launch.py \\
    [bag_dir:=/path/to/bags] \\
    [controller_protocol:=can|usart] \\
    [can_channel:=can0] [usart_port:=/dev/ttyUSB0] [usart_baud:=115200]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BASE_TOPICS = [
    "/camera/infra1/image_rect_raw",
    "/camera/infra1/camera_info",
    "/camera/infra2/image_rect_raw",
    "/camera/infra2/camera_info",
    "/camera/imu",
    "/camera_d405/color/image_raw",
    "/camera_d405/color/camera_info",
]


def _read_camera_serials() -> dict[str, str]:
    """Parse camera_serials.conf from the bringup config directory."""
    pkg_share = get_package_share_directory("umi_dex_bringup")
    conf_path = os.path.join(pkg_share, "config", "camera_serials.conf")
    serials: dict[str, str] = {}
    try:
        with open(conf_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if "=" in line:
                    key, val = line.split("=", 1)
                    serials[key.strip()] = val.strip()
    except FileNotFoundError:
        pass
    return serials


def _hand_topic_for(protocol: str) -> str:
    return "/hand/usart_raw" if protocol == "usart" else "/hand/can_raw"


def _build_recorder(context, *_args, **_kwargs):
    protocol = LaunchConfiguration("controller_protocol").perform(context)
    hand_topic = _hand_topic_for(protocol)
    topics = BASE_TOPICS + [hand_topic, "/session/episode"]

    return [
        Node(
            package="umi_dex_bringup",
            executable="interactive_capture_node",
            name="interactive_capture",
            output="screen",
            parameters=[{
                "bag_dir": LaunchConfiguration("bag_dir"),
                "base_name": "capture",
                "warmup_duration_s": LaunchConfiguration("warmup_duration_s"),
                "episode_topic": "/session/episode",
                "topics": topics,
            }],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    serials = _read_camera_serials()
    d455_serial = serials.get("d455_serial", "")
    d405_serial = serials.get("d405_serial", "")

    pkg_share = get_package_share_directory("umi_dex_bringup")
    launch_dir = os.path.join(pkg_share, "launch")

    return LaunchDescription([
        DeclareLaunchArgument("controller_protocol", default_value="can",
                              description="Controller link: 'can' or 'usart'"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        DeclareLaunchArgument("usart_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("usart_baud", default_value="115200"),
        DeclareLaunchArgument("bag_dir", default_value="outputs"),
        DeclareLaunchArgument("warmup_duration_s", default_value="15.0"),

        # D455 stereo IR + IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "d455.launch.py")
            ),
            launch_arguments={"serial_no": d455_serial}.items(),
        ),

        # D405 color
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "d405.launch.py")
            ),
            launch_arguments={"serial_no": d405_serial}.items(),
        ),

        # Controller raw publisher (CAN or USART)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "controller.launch.py")
            ),
            launch_arguments={
                "controller_protocol": LaunchConfiguration("controller_protocol"),
                "can_channel": LaunchConfiguration("can_channel"),
                "usart_port": LaunchConfiguration("usart_port"),
                "usart_baud": LaunchConfiguration("usart_baud"),
            }.items(),
        ),

        # Interactive capture node (topic list depends on protocol)
        OpaqueFunction(function=_build_recorder),
    ])
