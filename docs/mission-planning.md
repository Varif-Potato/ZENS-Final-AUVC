# Mission Planning

## Task Checklist

- [ ] Calibrate camera: `ros2 run cv calibrate_camera data/calib_images/`
- [ ] Verify AprilTag detection: `ros2 launch bringup cv.launch.py`
- [ ] Tune depth PID gains in `bringup/config/params.yaml`
- [ ] Tune heading PID gains in `bringup/config/params.yaml`
- [ ] Test flashing behavior with fake camera feed
- [ ] Pool test: dry-run all_systems.launch.py on tether

## Testing Workflow

```bash
# 1. Source environment
source ros_ws/install/setup.bash

# 2. Test CV with a video file
ros2 launch cv test_detection.launch.py

# 3. Test PID in simulation
ros2 launch bringup control.launch.py

# 4. Full system
ros2 launch bringup all_systems.launch.py
```
