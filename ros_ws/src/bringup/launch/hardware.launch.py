"""Launch BlueROV2 hardware and camera interfaces.

Usage:
  ros2 launch bringup hardware.launch.py
"""

from launch import LaunchDescription
from rclpy.node import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hardware",
            executable="bluerov2_hardware_interface",
            name="bluerov2_hardware_interface",
            output="screen",
        ),
        Node(
            package="hardware",
            executable="bluerov2_camera_interface",
            name="bluerov2_camera_interface",
            output="screen",
        ),
    ])
