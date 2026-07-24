*---
name: ros2-auvc
description: Use this skill when working on ROS2 packages, nodes, topics, services, actions, launch files, or debugging for the MIT Beaver Works AUV Challenge. Covers workspace setup, publisher/subscriber patterns, sensor and thruster integration, QoS settings, and common ROS2 CLI commands for underwater robotics.
---

# ROS2 for the AUV Challenge 🐠

You are helping a student build ROS2 nodes for an autonomous underwater vehicle (AUV). Water is unforgiving — no WiFi to save you, no do-overs mid-mission — so code must be robust, well-QoS'd, and simulator-tested before it ever touches a pool. Be precise, be safe, and don't let anyone `rm -rf` the workspace.

## When to use this skill

Trigger this skill for tasks like:
- Setting up or troubleshooting a ROS2 workspace (`colcon build`, `source install/setup.bash`)
- Writing publisher/subscriber nodes for sensors (IMU, DVL, depth sensor, camera) or actuators (thrusters, fins)
- Wiring up services/actions for mission control (e.g. "start mission", "surface now")
- Writing or debugging launch files
- Diagnosing dropped messages, QoS mismatches, or TF issues
- Reviewing student code for ROS2 best practices before a pool test

## Workspace setup

```bash
mkdir -p ~/auv_ws/src && cd ~/auv_ws
colcon build --symlink-install
source install/setup.bash
```

Always `source install/setup.bash` in every new terminal — "it's not compiling" is often "you forgot to source," 90% of the time.

Create a package:

```bash
cd ~/auv_ws/src
ros2 pkg create --build-type ament_python my_auv_node --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

Use `ament_cmake` instead of `ament_python` for C++ packages (recommended for anything running on tight control loops, like thruster mixing).

## Node template (Python)

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import FluidPressure

class DepthSensorNode(Node):
    def __init__(self):
        super().__init__('depth_sensor_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(FluidPressure, 'depth/pressure', qos)
        self.timer = self.create_timer(0.05, self.publish_depth)  # 20 Hz

    def publish_depth(self):
        msg = FluidPressure()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.fluid_pressure = self.read_sensor()
        self.pub.publish(msg)

    def read_sensor(self) -> float:
        return 101325.0  # replace with real driver read

def main():
    rclpy.init()
    node = DepthSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## QoS cheat sheet for AUVs

| Data type | Reliability | History | Why |
|---|---|---|---|
| Sensor streams (IMU, depth, DVL) | Best effort | Keep last, small depth | Fresh data matters more than every sample; don't back up the pipe |
| Mission commands / state changes | Reliable | Keep last | You must not drop a "surface now" |
| Camera / vision frames | Best effort | Keep last 1-2 | Old frames are useless, latency kills |
| Actuator commands (thrusters) | Reliable, low depth | Keep last 1 | Every command matters, but never act on stale ones |

Mismatched QoS between publisher and subscriber is the silent killer of "why isn't my topic showing data" — check with `ros2 topic info <topic> -v`.

## Launch file (Python)

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_auv_node', executable='depth_sensor_node', name='depth_sensor', output='screen'),
        Node(package='my_auv_node', executable='thruster_mixer', name='thruster_mixer', output='screen'),
        Node(package='my_auv_node', executable='mission_control', name='mission_control', output='screen',
             parameters=[{'max_depth_m': 3.0}]),
    ])
```

Run with `ros2 launch my_auv_node auv.launch.py`.

## Debugging commands (memorize these)

```bash
ros2 node list                  # what's actually running
ros2 topic list                 # available topics
ros2 topic hz /depth/pressure   # is it actually publishing at rate?
ros2 topic info /depth/pressure -v   # QoS mismatch detective work
ros2 topic echo /cmd_vel        # see raw messages
ros2 service list               # available services
ros2 action list                # available actions
ros2 bag record -a              # record everything before a pool run — always
```

### About `rqt_graph` on a headless Pi

The autonomy Pi has no video output, and `rqt_graph` is a Qt GUI app — it will not run there. Don't bother trying to launch it over SSH.

Instead:
- Run all ROS2 nodes headless on the Pi as normal.
- On your laptop (same network, same `ROS_DOMAIN_ID` as the Pi), run `rqt_graph` there — it discovers and visualizes the Pi's live node/topic graph remotely, no X-forwarding needed.
- For anything you need directly on the Pi, stick to CLI-only tools: `ros2 topic hz`, `ros2 topic echo`, `ros2 node list`, `ros2 topic info -v`.

```bash
export ROS_DOMAIN_ID=<same as Pi>   # on your laptop
rqt_graph
```

## AUV-specific gotchas

- Use `sensor_msgs/FluidPressure` or a custom depth message, not raw floats — future-you doing sensor fusion will thank you.
- Always timestamp sensor messages via `header.stamp` — TF and EKF nodes (e.g. `robot_localization`) silently break without it.
- Thruster mixer nodes should clamp output and fail-safe to zero on stale command timeout — a stuck thruster underwater is a bad day.
- Test in simulation (Gazebo/UUV Simulator or the course sim) before any pool test — debugging over a tether is miserable.
- Namespace your nodes per-vehicle (`/auv1/...`) if multiple AUVs share a network — nobody wants friendly fire.

## Code review checklist

- [ ] Node destroys cleanly (`destroy_node()`, no dangling timers)
- [ ] QoS explicitly set, not left to defaults, for anything safety-critical
- [ ] Timeouts/fail-safes on all actuator command subscriptions
- [ ] No blocking calls inside callbacks (use callback groups/executors if needed)
- [ ] Launch file parameterizes anything a student might need to tune (depth limits, gains)
