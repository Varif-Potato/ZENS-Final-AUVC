from setuptools import find_packages, setup
import os
from glob import glob

package_name = "cv"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ngt",
    maintainer_email="nthrasher92973@gmail.com",
    description="AprilTag detection node for BlueROV2",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "apriltag_detector_node = cv.nodes.detector_node:main",
            "fake_camera_node = cv.nodes.fake_camera_node:main",
            "image_saver_node = cv.nodes.image_saver_node:main",
            "calibrate_camera = cv.scripts.calibrate_camera:main",
            "detect_from_image = cv.scripts.detect_from_image:main",
            "tag_heading_node = cv.nodes.tag_heading_node:main",
        ],
    },
)
