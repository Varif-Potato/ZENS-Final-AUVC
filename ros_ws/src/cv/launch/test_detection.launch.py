import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("cv"), "config"
    )

    return LaunchDescription([
        Node(
            package="cv",
            executable="fake_camera_node",
            name="fake_camera",
            parameters=[{"source": "0", "fps": 15.0}],
        ),
        Node(
            package="cv",
            executable="apriltag_detector_node",
            name="apriltag_detector",
            parameters=[{"config_dir": config_dir, "camera_topic": "/camera"}],
            output="screen",
        ),
    ])
