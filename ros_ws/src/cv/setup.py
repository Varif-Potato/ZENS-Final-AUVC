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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py") + glob("launch/*.launch.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ngt",
    maintainer_email="nthrasher92973@gmail.com",
    description="Computer vision pipeline: AprilTag + YOLO detection for BlueROV2 (AUVC)",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "detection_node = cv.nodes.detection_node:main",
            "calibrate_camera = cv.scripts.calibrate_camera:main",
            "tag_heading_node = cv.nodes.tag_heading_node:main",
            "web_streamer_node = cv.nodes.web_streamer_node:main",
        ],
    },
)
