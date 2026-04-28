"""Play back a recorded bag.

Usage:
  ros2 launch umi_dex_bringup playback.launch.py bag:=/path/to/bag_dir
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="Path to the bag directory"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("start_offset", default_value="0.0"),
        DeclareLaunchArgument("loop", default_value="false"),

        # Non-loop playback
        ExecuteProcess(
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("loop"), "' == 'false'"])
            ),
            cmd=[
                "ros2", "bag", "play",
                LaunchConfiguration("bag"),
                "--clock", "100",
                "--rate", LaunchConfiguration("rate"),
                "--start-offset", LaunchConfiguration("start_offset"),
            ],
            output="screen",
        ),

        # Looped playback
        ExecuteProcess(
            condition=IfCondition(
                PythonExpression(["'", LaunchConfiguration("loop"), "' == 'true'"])
            ),
            cmd=[
                "ros2", "bag", "play",
                LaunchConfiguration("bag"),
                "--clock", "100",
                "--rate", LaunchConfiguration("rate"),
                "--start-offset", LaunchConfiguration("start_offset"),
                "--loop",
            ],
            output="screen",
        ),
    ])
