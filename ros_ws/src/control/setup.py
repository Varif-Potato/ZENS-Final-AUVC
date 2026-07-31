from setuptools import find_packages, setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ngt",
    maintainer_email="nthrasher92973@gmail.com",
    description="PID controllers for depth hold and heading hold",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "heading_pid = control.heading_pid:main",
            "depth_pid = control.depth_pid:main",
            "self_arm = control.self_arm:main",
            "self_disarm = control.self_disarm:main",
            "finalController = control.finalController:main",
        ],
    },
)
