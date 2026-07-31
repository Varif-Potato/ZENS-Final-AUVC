import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
 
class FinalController(Node):
    def __init__(self):
        super().__init__("final_controller")
 
        """ ---------------- Robot State ----------------"""
        self.currentAngle = 0.0
        self.currentSpeed = 0
 
        """ YOLO """
        self.auv_detected = False
        self.auv_detection = None
 
        """ AprilTag """
        self.tag_detected = False
        self.tag_id = None
        self.tag_distance = None
        self.tag_translation = None
        self.tag_center = None
 
        """ Flashing state (non-blocking) """
        self._initial_flash_rounds = 5
        self.flashRounds = self._initial_flash_rounds
        self.flashing = False
        self.flash_on = False
        self.last_flash_time = 0.0
        self.flash_interval = 0.25
        self.completed_flashing = False
 
        """ Search / spin state """
        self.search_heading = 0.0
        self.search_step = 2.0
 

        """ Startup reverse state """
        self.startup_reverse = True
        self.reverse_ticks = 80      # 80 × 0.1 s = 5 seconds

        """ configurable camera width (used to compute image center) """
        self.camera_width = int(self.declare_parameter("camera_width", 640).value)
 
        """ ---------------- Subscribers ---------------- """
        self.yolo_sub = self.create_subscription(
            String,
            "/yolo/detections",
            self.getYOLO,
            10
        )
 
        self.aprilTagSub = self.create_subscription(
            String,
            "/apriltags/detections",
            self.apriltag_callback,
            10
        )
 
        self.angle_sub = self.create_subscription(
            Float32,
            "/current_heading",
            self.getAngle,
            10
        )
 
        self.for_sub = self.create_subscription(
            Float32,
            "/current_for",
            self.getSpeed,
            10
        )
 
        """ ---------------- Publishers ---------------- """
        self.for_pub = self.create_publisher(
            Float32,
            "/target_speed",
            10
        )
 
        self.angle_pub = self.create_publisher(
            Float32,
            "/target_angle",
            10
        )
 
        self.lightPub = self.create_publisher(
            Int16,
            "/target_lights",
            10
        )
 
        self.timer = self.create_timer(
            0.1,
            self.scanning
        )
 
    """ ---------------------------------------------------- """
 
    def getYOLO(self, msg):
        """
        Expecting JSON list of detections shaped like the repo's YOLO output:
        { "class_id": ..., "class_name": "auv", "confidence": ..., "bbox": [x1,y1,x2,y2] }
        """
        try:
            detections = json.loads(msg.data)
 
            self.auv_detected = False
            self.auv_detection = None
 
            for detection in detections:
                cls = detection.get("class_name") or detection.get("class") or detection.get("label")
                if cls == "auv":
                    # store the detection as-is (we compute center later)
                    self.auv_detected = True
                    self.auv_detection = detection
                    confidence = detection.get("confidence", 0.0)
                    self.get_logger().info(f"AUV detected ({confidence:.2f})")
                    break
 
        except Exception as e:
            self.get_logger().error(f"Failed to parse YOLO: {e}")
 
    """ ---------------------------------------------------- """
 
    def apriltag_callback(self, msg):
        try:
            detections = json.loads(msg.data)
 
            # If no tags, clear tag state and reset flashing to allow future flashes
            if len(detections) == 0:
                self.tag_detected = False
                self.tag_id = None
                self.tag_distance = None
                self.tag_translation = None
                self.tag_center = None
                # Reset flash state so next close tag will trigger flashing again
                self.flashRounds = self._initial_flash_rounds
                self.flashing = False
                self.flash_on = False
                self.last_flash_time = 0.0
                self.completed_flashing = False
                return
 
            tag = detections[0]
 
            self.tag_detected = True
            self.tag_id = tag.get("id")
            self.tag_distance = tag.get("distance")
            self.tag_translation = tag.get("translation")
            self.tag_center = tag.get("center")
 
            if self.tag_distance is not None:
                self.get_logger().info(f"Tag {self.tag_id} at {self.tag_distance:.2f} m")
 
        except Exception as e:
            self.get_logger().error(f"Failed to parse AprilTag: {e}")
 
    """ ---------------------------------------------------- """
 
    def getSpeed(self, msg):
        self.currentSpeed = msg.data
 
    def getAngle(self, msg):
        self.currentAngle = msg.data
 
    """ ---------------------------------------------------- """
 
    def Publights(self, pwm):
        msg = Int16()
        msg.data = pwm
        self.lightPub.publish(msg)
 
    """ ---------------------------------------------------- """
 
    def _stop_motors(self):
        speed_msg = Float32()
        speed_msg.data = 0
        self.for_pub.publish(speed_msg)
        self.get_logger().debug("Motors stopped.")
 
    """ ---------------------------------------------------- """
 
    def scanning(self):
        heading_msg = Float32()
        speed_msg = Float32()

        # Startup reverse before any other behavior
        if self.startup_reverse:
            heading_msg.data = float(self.currentAngle)
            speed_msg.data = -300.0

            self.angle_pub.publish(heading_msg)
            self.for_pub.publish(speed_msg)

            self.reverse_ticks -= 1
            self.get_logger().info(
                f"Backing up... {self.reverse_ticks} ticks remaining"
            )

            if self.reverse_ticks <= 0:
                self.startup_reverse = False
                self.get_logger().info("Startup reverse complete.")

            return

 
        # 1) SEARCH MODE: nothing detected and not flashing
        if not self.tag_detected and not self.auv_detected and not self.flashing:
            # Advance an independent search heading each tick so the robot
            # actually sweeps around instead of chasing a target that moves
            # with currentAngle (which just oscillates in place).
            self.search_heading = (self.search_heading + self.search_step) % 360
            heading_msg.data = float(self.search_heading)
            self.angle_pub.publish(heading_msg)
 
            # Keep forward speed at zero so this is a pure spin-in-place.
            speed_msg.data = 0.0
            self.for_pub.publish(speed_msg)
 
            self.get_logger().info(f"Searching... heading {heading_msg.data}")
            return
 
        # 2) FLASHING MODE (highest priority if tag within threshold)
        if (
            self.tag_detected
            and self.tag_distance is not None
            and self.tag_distance <= 1.0
            and not self.completed_flashing
        ):
            now = self.get_clock().now().nanoseconds / 1e9
 
            if not self.flashing:
                self.flashing = True
                self.flash_on = False
                self.last_flash_time = now - self.flash_interval
                # Ensure motors are stopped while flashing
                self._stop_motors()
                self.get_logger().info("Starting flashing sequence.")
 
            # toggle on/off based on interval
            if now - self.last_flash_time >= self.flash_interval:
                if self.flash_on:
                    # end of ON phase -> turn lights off, count a round
                    self.Publights(1100)
                    self.flash_on = False
                    self.flashRounds -= 1
                    self.get_logger().info(f"Flashed ({self._initial_flash_rounds - self.flashRounds}/{self._initial_flash_rounds})")
                else:
                    # start ON phase
                    self.Publights(1900)
                    self.flash_on = True
 
                self.last_flash_time = now
 
            # if finished all rounds, finalize
            if self.flashRounds <= 0:
                self.flashing = False
                self.completed_flashing = True
                self.Publights(1100)
                # ensure motors are stopped after flashing
                self._stop_motors()
                self.get_logger().info("Finished flashing.")
            return
 
        # 3) AUV FOLLOW MODE (YOLO) - only if AUV detected and no AprilTag (tag has priority)
        if self.auv_detected and not self.tag_detected and not self.completed_flashing:
            det = self.auv_detection or {}
            bbox = det.get("bbox")
 
            center = None
            if bbox and isinstance(bbox, (list, tuple)) and len(bbox) == 4:
                # repo's YOLO output uses pixel bbox [x1, y1, x2, y2]
                x1, y1, x2, y2 = bbox
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                center = (cx, cy)
            else:
                # if a 'center' is provided use that
                _center = det.get("center")
                if _center and isinstance(_center, (list, tuple)) and len(_center) >= 2:
                    center = (_center[0], _center[1])
 
            if center is not None:
                try:
                    cx, _cy = center
                    image_center = float(self.camera_width) / 2.0
                    error = cx - image_center
                    # simple proportional-ish steering: adjust by sign of error, scaled step
                    step = 5
                    if error > 20:
                        heading_msg.data = int((self.currentAngle + step) % 360)
                    elif error < -20:
                        heading_msg.data = int((self.currentAngle - step) % 360)
                    else:
                        heading_msg.data = int(self.currentAngle)
 
                    speed_msg.data = 300
                    self.angle_pub.publish(heading_msg)
                    self.for_pub.publish(speed_msg)
 
                    self.get_logger().info("Following AUV (YOLO).")
                except Exception as e:
                    self.get_logger().error(f"Error using YOLO center: {e}")
            else:
                # fallback: rotate slowly to search for better detection
                target_heading = int((self.currentAngle + 30) % 360)
                heading_msg.data = target_heading
                self.angle_pub.publish(heading_msg)
                self.get_logger().info("AUV detected but no center/bbox info — rotating to search.")
            return
 
        # 4) APRILTAG APPROACH MODE
        if self.tag_detected:
            if self.tag_distance is not None and self.tag_distance > 1.0:
                speed_msg.data = 300
                if self.tag_center is not None:
                    try:
                        x, y = self.tag_center
                        image_center = float(self.camera_width) / 2.0
                        error = x - image_center
                        if error > 20:
                            heading_msg.data = int((self.currentAngle + 5) % 360)
                        elif error < -20:
                            heading_msg.data = int((self.currentAngle - 5) % 360)
                        else:
                            heading_msg.data = int(self.currentAngle)
                        self.angle_pub.publish(heading_msg)
                    except Exception as e:
                        self.get_logger().error(f"Invalid tag_center: {e}")
 
                self.for_pub.publish(speed_msg)
                self.get_logger().info("Approaching AprilTag...")
                return
 
        # Default: nothing matched — safe no-op (motors remain at last commanded state)
 
def main(args=None):
    rclpy.init(args=args)
    node = FinalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ensure lights off and motors stopped on shutdown
        node.Publights(1100)
        node._stop_motors()
        node.destroy_node()
        rclpy.shutdown()
 
if __name__ == "__main__":
    main()