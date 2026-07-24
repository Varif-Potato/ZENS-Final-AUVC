import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FakeCameraNode(Node):
    def __init__(self):
        super().__init__("fake_camera")
        self.pub = self.create_publisher(Image, "/camera", 10)
        self.bridge = CvBridge()

        source = (
            self.declare_parameter("source", "0").value
        )
        if source.isdigit():
            self.cap = cv2.VideoCapture(int(source))
        else:
            self.cap = cv2.VideoCapture(source)

        fps = self.declare_parameter("fps", 15.0).value
        self.timer = self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            f"Publishing from '{source}' at ~{fps} fps on /camera"
        )

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("End of video stream — looping")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraNode()
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
