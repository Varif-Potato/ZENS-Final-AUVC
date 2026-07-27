import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String


class TagHeadingNode(Node):
    def __init__(self):
        super().__init__("tag_heading")

        self.fx = self.declare_parameter("fx", 554.0).value
        self.cx = self.declare_parameter("cx", 320.0).value
        self.image_width = self.declare_parameter("image_width", 640).value
        self.tag_id = self.declare_parameter("tag_id", -1).value
        self.active = self.declare_parameter("active", True).value

        self.current_heading = None

        self.target_pub = self.create_publisher(Int16, "/target_angle", 10)

        self.heading_sub = self.create_subscription(
            Int16, "/heading", self.heading_callback, 10
        )
        self.detections_sub = self.create_subscription(
            String, "/apriltags/detections", self.detections_callback, 10
        )

        self.get_logger().info(
            f"TagHeadingNode active — fx={self.fx}, cx={self.cx}, "
            f"image_width={self.image_width}, tag_id={self.tag_id}"
        )

    def heading_callback(self, msg):
        self.current_heading = msg.data

    def detections_callback(self, msg):
        if not self.active:
            return
        if self.current_heading is None:
            return

        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Failed to parse detections JSON")
            return

        if not detections:
            return

        centroid, ids = self._detection_centroid(detections)
        if centroid is None:
            return

        offset_x = centroid[0] - self.cx
        yaw_offset_deg = math.degrees(math.atan2(offset_x, self.fx))
        target_angle = int((self.current_heading + yaw_offset_deg) % 360)

        self.target_pub.publish(Int16(data=target_angle))
        self.get_logger().info(
            f"{len(ids)} tag(s) ids={ids}: "
            f"centroid=({centroid[0]:.1f}, {centroid[1]:.1f}), "
            f"offset_x={offset_x:.1f}px, yaw_offset={yaw_offset_deg:.1f}°, "
            f"heading={self.current_heading}° → target={target_angle}°",
            throttle_duration_sec=0.5,
        )

    def _detection_centroid(self, detections):
        centers = []
        ids = []

        for d in detections:
            if self.tag_id >= 0 and d["id"] != self.tag_id:
                continue
            center = d.get("center")
            if center is None:
                continue
            centers.append(center)
            ids.append(d["id"])

        if not centers:
            return None, []

        centroid_x = sum(c[0] for c in centers) / len(centers)
        centroid_y = sum(c[1] for c in centers) / len(centers)
        return (centroid_x, centroid_y), ids


def main(args=None):
    rclpy.init(args=args)
    node = TagHeadingNode()
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
