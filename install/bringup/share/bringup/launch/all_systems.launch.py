"""Master launch file — runs all AUVC subsystems.

Starts hardware interfaces, control (PID), computer vision (AprilTag),
and the flashing lights behavior in a single command.

Usage:
  ros2 launch bringup all_systems.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")

    return LaunchDescription([
        # Hardware and camera interfaces
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "hardware.launch.py")
            ),
        ),

        # PID controllers (depth hold + heading hold)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "control.launch.py")
            ),
        ),

        # AprilTag computer vision
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "cv.launch.py")
            ),
        ),

        # Flashing lights behavior (auto-starts when tag < 1m)
        Node(
            package="flashing",
            executable="flashing_node",
            name="flashing_node",
            output="screen",
        ),
    ])
