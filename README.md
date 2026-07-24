# ZENS-Final-AUVC

AUVC final project — BlueROV2 with PID control, AprilTag computer vision,
and mission-specific behaviors.

## Structure

| Directory | Contents |
|-----------|----------|
| `ros_ws/src/bringup/` | Master + granular launch files, shared config |
| `ros_ws/src/control/` | Depth PID, heading PID, arm/disarm |
| `ros_ws/src/cv/` | AprilTag detection, calibration, camera tools |
| `ros_ws/src/flashing/` | Lights flashing on tag proximity |
| `ros_ws/src/hardware/` | BlueROV2 MAVLink + camera GStreamer interface |
| `ros_ws/src/msgs/` | Custom message definitions |
| `calibration/` | Camera & sensor calibration tools |
| `simulation/` | Gazebo worlds and sim launch scripts |
| `analysis/` | Bag recording, post-mission notebooks |
| `notebooks/` | Experiment notebooks |
| `assets/` | Test videos and images |
| `scripts/` | Environment setup utilities |
| `docs/` | Architecture docs, mission planning |

## Quick Start

```bash
# Build
cd ros_ws
colcon build --symlink-install
source install/setup.bash

# Run everything
ros2 launch bringup all_systems.launch.py

# Or run individual subsystems:
ros2 launch bringup control.launch.py    # PID only
ros2 launch bringup cv.launch.py         # CV only
ros2 launch bringup hardware.launch.py   # Hardware only
```

## Launch Files

| Command | What it runs |
|---------|--------------|
| `ros2 launch bringup all_systems.launch.py` | Everything |
| `ros2 launch bringup control.launch.py` | depth_pid + heading_pid |
| `ros2 launch bringup cv.launch.py` | AprilTag detector |
| `ros2 launch bringup hardware.launch.py` | MAVLink + camera interface |
