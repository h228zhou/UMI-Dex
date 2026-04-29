"""Launch D455 stereo IR + IMU via the realsense2_camera ROS2 wrapper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="",
                              description="D455 serial number (empty=auto)"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("realsense2_camera"),
                    "launch", "rs_launch.py",
                ])
            ),
            launch_arguments={
                "serial_no": LaunchConfiguration("serial_no"),
                "device_type": "D455",
                "camera_name": "camera",
                "camera_namespace": "",
                "enable_infra1": "true",
                "enable_infra2": "true",
                "depth_module.infra_profile": "848x480x30",
                "enable_color": "false",
                "enable_depth": "false",
                "enable_gyro": "true",
                "enable_accel": "true",
                "gyro_fps": "200",
                "accel_fps": "200",
                "unite_imu_method": "2",
                "enable_sync": "true",
                "initial_reset": "false",
                "depth_module.enable_auto_exposure": "true",
            }.items(),
        ),
    ])
