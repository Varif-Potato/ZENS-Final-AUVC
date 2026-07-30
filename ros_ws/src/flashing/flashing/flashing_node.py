import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16


class FlashingNode(Node):
    def __init__(self):
        super().__init__("flashing_node")

        self.declare_parameter("flash_distance", 1.0)
        self.declare_parameter("flash_interval", 0.5)

        self.flash_distance = self.get_parameter("flash_distance").value
        self.flash_interval = self.get_parameter("flash_interval").value

        self.tag_near = False
        self.lights_on = False
        self.flash_timer = None

        self.detections_sub = self.create_subscription(
            String,
            "/apriltags/detections",
            self.detections_callback,
            10,
        )

        self.lights_pub = self.create_publisher(
            Int16,
            "/target_lights",
            10,
        )

        self.get_logger().info(
            f"FlashingNode started — flash_distance={self.flash_distance}m, "
            f"flash_interval={self.flash_interval}s"
        )

    def detections_callback(self, msg):
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        was_near = self.tag_near
        self.tag_near = False

        for tag in detections:
            center = tag.get("center")
            if center:
                cx, cy = center
                distance = math.sqrt(cx**2 + cy**2)
                if distance < self.flash_distance:
                    self.tag_near = True
                    self.get_logger().info(
                        f"Tag {tag['id']} within {self.flash_distance}m — flashing"
                    )
                    break

        if self.tag_near and not was_near:
            self._start_flashing()
        elif not self.tag_near and was_near:
            self._stop_flashing()

    def _start_flashing(self):
        if self.flash_timer is not None:
            self.flash_timer.cancel()
        self.lights_on = True
        self._publish_lights()
        self.flash_timer = self.create_timer(self.flash_interval, self._toggle_lights)
        self.get_logger().info("Flashing started")

    def _stop_flashing(self):
        if self.flash_timer is not None:
            self.flash_timer.cancel()
            self.flash_timer = None
        self.lights_on = False
        self._publish_lights()
        self.get_logger().info("Flashing stopped — lights off")

    def _toggle_lights(self):
        self.lights_on = not self.lights_on
        self._publish_lights()

    def _publish_lights(self):
        msg = Int16()
        msg.data = 1900 if self.lights_on else 1100
        self.lights_pub.publish(msg)

    def destroy_node(self):
        if self.flash_timer is not None:
            self.flash_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FlashingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
