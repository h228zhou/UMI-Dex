"""Full capture pipeline: D455 + D405 + CAN controller + interactive recorder.

Usage:
  ros2 launch umi_dex_bringup capture.launch.py [bag_dir:=/path/to/bags]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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


def generate_launch_description() -> LaunchDescription:
    serials = _read_camera_serials()
    d455_serial = serials.get("d455_serial", "")
    d405_serial = serials.get("d405_serial", "")

    pkg_share = get_package_share_directory("umi_dex_bringup")
    launch_dir = os.path.join(pkg_share, "launch")

    topics = [
        "/camera/infra1/image_rect_raw",
        "/camera/infra1/camera_info",
        "/camera/infra2/image_rect_raw",
        "/camera/infra2/camera_info",
        "/camera/imu",
        "/camera_d405/color/image_raw",
        "/camera_d405/color/camera_info",
        "/hand/can_raw",
        "/session/episode",
    ]

    return LaunchDescription([
        DeclareLaunchArgument("can_channel", default_value="can0"),
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

        # CAN raw frame publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "controller.launch.py")
            ),
            launch_arguments={
                "can_channel": LaunchConfiguration("can_channel"),
            }.items(),
        ),

        # Interactive capture node
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
    ])
