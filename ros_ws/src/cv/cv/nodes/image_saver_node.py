from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__("image_saver")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.declare_parameter("input_topic", "/apriltags/annotated").value,
            self.callback,
            10,
        )
        self.srv = self.create_service(
            Trigger,
            "~/save",
            self.save_callback,
        )

        self.latest_frame = None
        self.save_counter = 0
        self.output_dir = Path(
            self.declare_parameter("output_dir", ".").value
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f"Saving frames to {self.output_dir} — "
            f"call '~/save' service to save"
        )

    def callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def save_callback(self, request, response):
        if self.latest_frame is None:
            response.success = False
            response.message = "No frame received yet"
            return response

        path = self.output_dir / f"frame_{self.save_counter:04d}.png"
        cv2.imwrite(str(path), self.latest_frame)
        self.save_counter += 1
        response.success = True
        response.message = f"Saved to {path}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
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
