import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import String
import time

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
        self.flashRounds = 5               # number of on/off cycles to perform
        self.flashing = False              # currently in flash sequence
        self.flash_on = False              # whether lights are currently on
        self.last_flash_time = 0.0         # last toggle time (seconds)
        self.flash_interval = 0.25         # seconds for each on/off phase
        self.completed_flashing = False    # once done, don't re-flash

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
            Int16,
            "/current_for",
            self.getSpeed,
            10
        )

        """ ---------------- Publishers ---------------- """
        self.for_pub = self.create_publisher(
            Int16,
            "/target_speed",
            10
        )

        self.angle_pub = self.create_publisher(
            Int16,
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
        try:
            detections = json.loads(msg.data)

            self.auv_detected = False
            self.auv_detection = None

            for detection in detections:
                # defensive: class field may be 'class' or 'label'
                cls = detection.get("class") or detection.get("label")
                if cls == "auv":
                    self.auv_detected = True
                    self.auv_detection = detection

                    confidence = detection.get("confidence", 0.0)

                    self.get_logger().info(
                        f"AUV detected ({confidence:.2f})"
                    )
                    break

        except Exception as e:
            self.get_logger().error(
                f"Failed to parse YOLO: {e}"
            )

    """ ---------------------------------------------------- """

    def apriltag_callback(self, msg):
        try:
            detections = json.loads(msg.data)

            self.tag_detected = False
            self.tag_id = None
            self.tag_distance = None
            self.tag_translation = None
            self.tag_center = None

            if len(detections) == 0:
                return

            tag = detections[0]

            self.tag_detected = True
            self.tag_id = tag.get("id")
            self.tag_distance = tag.get("distance")
            self.tag_translation = tag.get("translation")
            self.tag_center = tag.get("center")

            if self.tag_distance is not None:
                self.get_logger().info(
                    f"Tag {self.tag_id} at {self.tag_distance:.2f} m"
                )

        except Exception as e:
            self.get_logger().error(
                f"Failed to parse AprilTag: {e}"
            )

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
        speed_msg = Int16()
        speed_msg.data = 0
        self.for_pub.publish(speed_msg)
        # keep heading as-is (no new heading command)
        self.get_logger().debug("Motors stopped.")

    """ ---------------------------------------------------- """

    def scanning(self):
        heading_msg = Int16()
        speed_msg = Int16()

        # 1) SEARCH MODE: nothing detected
        if not self.tag_detected and not self.auv_detected and not self.flashing:
            # Rotate 180 degrees from current heading
            target_heading = int((self.currentAngle + 180) % 360)
            heading_msg.data = target_heading
            self.angle_pub.publish(heading_msg)
            self.get_logger().info(
                f"Searching... heading {target_heading}"
            )
            return

        # 2) FLASHING MODE (highest priority if tag within threshold)
        if (
            self.tag_detected
            and self.tag_distance is not None
            and self.tag_distance <= 1.0
            and not self.completed_flashing
        ):
            now = self.get_clock().now().nanoseconds / 1e9

            # if we are not currently in a flashing sequence, start it
            if not self.flashing:
                self.flashing = True
                self.flash_on = False
                self.last_flash_time = now - self.flash_interval  # trigger immediate toggle
                # stop motors when starting to flash
                self._stop_motors()
                self.get_logger().info("Starting flashing sequence.")

            # toggle on/off based on interval
            if now - self.last_flash_time >= self.flash_interval:
                if self.flash_on:
                    # turn lights off (end of one on period)
                    self.Publights(1100)
                    self.flash_on = False
                    # Count completed on/off cycle
                    self.flashRounds -= 1
                    self.get_logger().info(f"Flashed ({5 - self.flashRounds}/5)")
                else:
                    # turn lights on
                    self.Publights(1900)
                    self.flash_on = True

                self.last_flash_time = now

            # if finished all rounds, finalize
            if self.flashRounds <= 0:
                self.flashing = False
                self.completed_flashing = True
                self.Publights(1100)  # ensure lights off or default
                self.get_logger().info("Finished flashing.")
            return

        # 3) AUV FOLLOW MODE (YOLO) - only if AUV detected and no AprilTag (tag has priority)
        if self.auv_detected and not self.tag_detected and not self.completed_flashing:
            det = self.auv_detection or {}
            center = det.get("center")

            # if center is missing, try bbox formats:
            # common bbox variants: [x, y, w, h] or [x1, y1, x2, y2]
            if center is None:
                bbox = det.get("bbox")
                if bbox and isinstance(bbox, (list, tuple)) and len(bbox) == 4:
                    # heuristics: detect variant
                    x0, y0, x1, y1 = bbox
                    # If bbox looks like x,y,w,h (w,h small compared to coords), assume that:
                    if x1 < 1.0 and y1 < 1.0:
                        # normalized xywh (0-1)
                        cx = x0 + x1 / 2.0
                    else:
                        # assume x1,y1 are x2,y2
                        cx = (x0 + x1) / 2.0
                    # try to scale to pixel coordinates if normalized (assume width ~640)
                    if cx <= 1.0:
                        cx = int(cx * 640)
                    center = (cx, 0)

            if center is not None:
                try:
                    x, y = center
                    error = x - 320  # assume camera width 640 px, center at 320
                    if error > 20:
                        heading_msg.data = int((self.currentAngle + 20) % 360)
                    elif error < -20:
                        heading_msg.data = int((self.currentAngle - 20) % 360)
                    else:
                        heading_msg.data = int(self.currentAngle)

                    speed_msg.data = 300
                    self.angle_pub.publish(heading_msg)
                    self.for_pub.publish(speed_msg)

                    self.get_logger().info("Following AUV (YOLO).")
                except Exception as e:
                    self.get_logger().error(f"Error using YOLO center: {e}")
            else:
                # fallback: rotate slowly to search
                target_heading = int((self.currentAngle + 30) % 360)
                heading_msg.data = target_heading
                self.angle_pub.publish(heading_msg)
                self.get_logger().info("AUV detected but no center info — rotating to search.")
            return

        # 4) APRILTAG APPROACH MODE
        if self.tag_detected:
            # If tag is farther than 1m, approach
            if self.tag_distance is not None and self.tag_distance > 1.0:
                speed_msg.data = 300

                if self.tag_center is not None:
                    try:
                        x, y = self.tag_center
                        error = x - 320
                        if error > 20:
                            heading_msg.data = int((self.currentAngle + 20) % 360)
                        elif error < -20:
                            heading_msg.data = int((self.currentAngle - 20) % 360)
                        else:
                            heading_msg.data = int(self.currentAngle)

                        self.angle_pub.publish(heading_msg)
                    except Exception as e:
                        self.get_logger().error(f"Invalid tag_center: {e}")

                self.for_pub.publish(speed_msg)

                self.get_logger().info("Approaching AprilTag...")
                return

        # Default: if none of the above took action and we are not flashing, ensure motors are safe
        if not self.flashing and not self.completed_flashing:
            # no explicit command; could stop or hold; choose to do nothing here
            pass

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
