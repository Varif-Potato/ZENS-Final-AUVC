# ZENS-Final-AUVC

Beaverworks @ MIT Autonomous Underwater Vehicle Challenge final project
Controlling BlueROV2s entirely autonomously with PID controllers for movement, computer vision with object & AprilTag detection, and controlling the lights to flash when within one meter of the opposing ROV. 
## BWSI AUVC 2026 Rules
### Rules of Engagement of AUVC 2026
#### Arena and Setup
The competition takes place in a 25-yard pool using 3 adjacent lanes.
Lane boundaries are considered soft limits; leaving the 3-lane region risks disqualification at judge discretion.
Each team starts at opposite ends of the pool, centered in the middle lane.
Each AUV starts centered in the middle lane at opposite ends of the pool, facing the wall behind it (i.e., both AUVs initially face away from each other).
Any deviation from this starting orientation must be explicitly approved by judges before the round.
#### Pre-Round Conditions
A legal operating depth range $(d_min,d_max)$ is announced before each round.
AUVs must remain within this depth band for the entire run.
Teams may position and arm their AUV before the start signal, but no motion is allowed until the round begins.
#### Autonomy Constraint
At the start signal, each team may execute exactly one command (e.g., press Enter once to launch their script).
After execution, no further human interaction is allowed.
Any additional input (keyboard, mouse, SSH, etc.) results in immediate forfeiture unless both teams and judges agree to a restart.
#### Match Timing
Each round has a maximum duration of 5 minutes.
If no win condition is met within 5 minutes, the round is declared a draw unless tie-break rules are defined separately.
#### Sensing and Tagging
Each AUV has exactly 4 AprilTags: 2 on the bow, 2 on the aft.
No tags are present on the port or starboard sides.
Localization and distance estimation are based on AprilTag detections and logged for verification.
#### Win Condition
A round is won when one AUV “flashes” the opposing AUV within a distance of ≤1 meter.
“Flash” must be clearly observable in logs (e.g., LED trigger event with timestamp).
Distance is computed from AprilTag-based pose estimates at the time of the flash.
The flashing AUV must be within the legal depth range at the moment of the event.
#### Violations and Disqualification
Leaving the allowed depth range may result in disqualification.
Exiting the 3-lane competition area may result in disqualification.
Human intervention after start results in forfeiture.
Judges may invalidate a win if sensor logs are missing, inconsistent, or clearly erroneous.
#### Restarts
A round may be restarted only if:
Both teams agree, and
Judges determine there was an external fault (e.g., pool interference, infrastructure failure).
#### Hardware and Safety Constraints
No physical modifications to the AUV are allowed once inspection is complete. This includes changes to structure, buoyancy, sensor placement, wiring, or tag placement.
Teams may only adjust software parameters after inspection; any hardware change requires judge approval and may trigger re-inspection.
Thruster output is capped at a predefined maximum (specified before the round). Commands exceeding this limit must be clamped in software.
Judges may disqualify an AUV that exceeds the thruster limit or operates in a way deemed unsafe (e.g., excessive speed, instability, or risk of collision damage).

## Structure

| Directory | Contents |
|-----------|----------|
| `ros_ws/src/launcher/` | Master launch files |
| `ros_ws/src/control/` | Depth PID, heading PID, arm/disarm |
| `ros_ws/src/cv/` | AprilTag detection, YOLO detection, lights |
| `ros_ws/src/flashing/` | Lights flashing on tag proximity |
| `simulation/` | Gazebo simulation |
| `analysis/` | Bag recording, post-mission analysis |
| `assets/` | Videos, images, and YOLO training dataset |

## Launch Files

| Command | What it runs |
|---------|--------------|
| `ros2 launch launcher all_systems.launch.yaml` | Everything |
| `ros2 launch launcher control.launch.yaml` | depth_pid + heading_pid |
| `ros2 launch launcher cv.launch.yaml` | AprilTag detector |
| `ros2 launch launcher hardware.launch.yaml` | MAVLink + camera interface |
| `ros2 launch cv web_streamer.launch.py` | Flask API Live Video  |