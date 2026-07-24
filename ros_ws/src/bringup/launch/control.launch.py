"""Launch depth PID, heading PID, and forward motion controller.

The depth PID subscribes to /target_depth (Float32) and /target_for (Float32).
Publish to these topics at runtime to set depth and forward speed goals.

Usage:
  ros2 launch bringup control.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="control",
            executable="depth_pid",
            name="depth_pid",
            output="screen",
        ),
        Node(
            package="control",
            executable="heading_pid",
            name="heading_pid",
            output="screen",
        ),
    ])
