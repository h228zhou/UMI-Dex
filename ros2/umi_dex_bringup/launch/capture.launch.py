"""Hardware capture pipeline: D455 + D405 + controller (CAN or USART).

Drive recording from a second terminal via `ros2 run umi_dex_bringup record.sh`.
(The interactive recorder is not launched here because `ros2 launch` detaches
child stdin, which breaks the hotkey prompt.)

Usage:
  ros2 launch umi_dex_bringup capture.launch.py \\
    [controller_protocol:=can|usart] \\
    [can_channel:=can0] [usart_port:=/dev/ttyUSB0] [usart_baud:=115200]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    # Prefix non-empty all-digit serials with '_' — rs_launch.py's yaml.safe_load
    # would otherwise coerce them to int, which realsense2_camera rejects
    # (serial_no param is declared as string). The leading '_' is stripped
    # inside realsense_node_factory.cpp.
    def _as_serial(v: str) -> str:
        return f"_{v}" if v else ""
    d455_serial = _as_serial(serials.get("d455_serial", ""))
    d405_serial = _as_serial(serials.get("d405_serial", ""))

    pkg_share = get_package_share_directory("umi_dex_bringup")
    launch_dir = os.path.join(pkg_share, "launch")

    return LaunchDescription([
        DeclareLaunchArgument("controller_protocol", default_value="can",
                              description="Controller link: 'can' or 'usart'"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        DeclareLaunchArgument("usart_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("usart_baud", default_value="115200"),

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
    ])
