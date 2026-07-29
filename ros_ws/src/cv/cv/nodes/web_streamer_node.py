import os
os.environ["OPENCV_OPENCL_RUNTIME"] = ""

import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from flask import Flask, Response


class WebStreamerNode(Node):
    def __init__(self):
        super().__init__("web_streamer")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_jpeg = None

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)
        self.declare_parameter("topic", "/apriltags/annotated")
        self.declare_parameter("quality", 80)
        self.declare_parameter("max_fps", 15.0)

        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        topic = self.get_parameter("topic").value
        self.quality = self.get_parameter("quality").value
        self.max_fps = self.get_parameter("max_fps").value

        self.sub = self.create_subscription(Image, topic, self.callback, 10)

        self.flask_thread = threading.Thread(
            target=self._run_flask, args=(host, port), daemon=True
        )
        self.flask_thread.start()

        self.get_logger().info(
            f"Web streamer subscribing to '{topic}', "
            f"serving at http://{host}:{port}/"
        )

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        _, jpeg = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality]
        )
        with self.lock:
            self.latest_jpeg = jpeg.tobytes()

    def _get_jpeg(self):
        with self.lock:
            return self.latest_jpeg

    def _run_flask(self, host, port):
        app = Flask(__name__)

        @app.route("/")
        def index():
            return (
                "<html><body>"
                "<h1>BlueROV2 AprilTag Stream</h1>"
                '<img src="/stream" width="100%">'
                "</body></html>"
            )

        @app.route("/stream")
        def stream():
            def generate():
                interval = 1.0 / self.max_fps
                while True:
                    jpeg = self._get_jpeg()
                    if jpeg is not None:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                        )
                    time.sleep(interval)

            return Response(
                generate(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        app.run(host=host, port=port, threaded=True)


def main(args=None):
    rclpy.init(args=args)
    node = WebStreamerNode()
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