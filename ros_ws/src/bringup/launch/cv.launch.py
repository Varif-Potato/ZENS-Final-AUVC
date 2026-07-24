"""Launch AprilTag detection without arming the robot.

Runs the detector node which subscribes to /camera and publishes
annotated images to /apriltags/annotated and detection data to
/apriltags/detections.

Usage:
  ros2 launch bringup cv.launch.py
  ros2 launch bringup cv.launch.py camera_topic:=/rov1/camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("cv"), "config"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/camera",
            description="Camera image topic",
        ),
        Node(
            package="cv",
            executable="apriltag_detector_node",
            name="apriltag_detector",
            parameters=[{
                "config_dir": config_dir,
                "camera_topic": LaunchConfiguration("camera_topic"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
