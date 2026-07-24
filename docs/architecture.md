# ZENS-Final-AUVC Architecture

## Overview

ROS2 workspace for the MIT Beaver Works AUV Challenge (AUVC) — a BlueROV2
autonomous underwater vehicle with PID control, AprilTag computer vision,
and mission-specific behaviors.

## Package Layout

| Package | Purpose |
|---------|---------|
| `bringup` | Master + granular launch files, shared config |
| `control` | Depth PID, heading PID, arm/disarm |
| `cv` | AprilTag detection pipeline, camera calibration |
| `flashing` | Lights flashing behavior on tag proximity |
| `hardware` | BlueROV2 MAVLink interface, camera GStreamer pipeline |
| `msgs` | Custom ROS2 message definitions (TargetDepth) |

## Launch Hierarchy

```
all_systems.launch.py
  ├── hardware.launch.py       # MAVLink + camera interface
  ├── control.launch.py        # depth_pid + heading_pid
  ├── cv.launch.py             # AprilTag detector
  └── flashing_node            # lights on tag < 1m
```

## Topic Flow

```
/hardware (MAVLink)
  ├── /pressure        → control/depth_pid
  ├── /heading         → control/heading_pid
  └── /camera          → cv/apriltag_detector

/cv (AprilTag)
  ├── /apriltags/annotated    → visualization
  └── /apriltags/detections   → flashing/flashing_node

/control (PID)
  └── /manual_control  → hardware (MAVLink)

/flashing
  └── /lights          → hardware (LED control)
```
