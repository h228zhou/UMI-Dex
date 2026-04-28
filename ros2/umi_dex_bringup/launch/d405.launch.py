"""Launch D405 color stream via the realsense2_camera ROS2 wrapper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="",
                              description="D405 serial number (empty=auto)"),
        DeclareLaunchArgument("device_type", default_value="D405"),
        DeclareLaunchArgument("camera_name", default_value="camera_d405"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch", "rs_launch.py",
                ])
            ),
            launch_arguments={
                "serial_no": LaunchConfiguration("serial_no"),
                "device_type": LaunchConfiguration("device_type"),
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_namespace": "",
                "enable_color": "true",
                "color_width": "640",
                "color_height": "480",
                "color_fps": "30",
                "enable_depth": "false",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "enable_gyro": "false",
                "enable_accel": "false",
                "enable_sync": "true",
            }.items(),
        ),
    ])
