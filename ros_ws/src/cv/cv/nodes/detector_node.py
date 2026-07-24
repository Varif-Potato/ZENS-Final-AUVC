import os
os.environ["OPENCV_OPENCL_RUNTIME"] = ""

import threading
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from cv.detector import AprilTagDetector
from cv.camera import CameraCalibration
from cv.visualizer import detect_and_annotate
from cv.video_logger import VideoLogger

import ament_index_python


def _resolve_package_path(relative_path):
    share = ament_index_python.get_package_share_directory("cv")
    return os.path.join(share, relative_path)


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__("apriltag_detector")

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_frame = None
        self.latest_tags = []
        self.image_count = 0

        config_dir = (
            self.declare_parameter("config_dir", "").value
            or _resolve_package_path("config")
        )
        detector_params_path = os.path.join(
            config_dir, "detector_params.yaml"
        )
        camera_info_path = os.path.join(
            config_dir, "camera_info.yaml"
        )

        detector_params = self._load_yaml(detector_params_path)
        self.detector = AprilTagDetector.from_params(
            detector_params.get("apriltag_detector", {})
        )

        if os.path.exists(camera_info_path):
            self.camera = CameraCalibration.from_yaml(camera_info_path)
            self.get_logger().info(f"Loaded camera calibration from {camera_info_path}")
        else:
            self.camera = None
            self.get_logger().warn(
                f"No camera calibration at {camera_info_path} — "
                "pose estimation disabled"
            )

        camera_topic = self.declare_parameter("camera_topic", "/camera").value
        self.sub = self.create_subscription(
            Image, camera_topic, self.callback, 10
        )
        self.annotated_pub = self.create_publisher(
            Image, "/apriltags/annotated", 10
        )
        self.detections_pub = self.create_publisher(
            String, "/apriltags/detections", 10
        )

        self.get_logger().info(
            f"Subscribed to '{camera_topic}', "
            f"publishing annotated -> /apriltags/annotated, "
            f"detections -> /apriltags/detections"
        )

        self.create_timer(1.0, self.status_timer)

    def _load_yaml(self, path):
        import yaml
        with open(path) as f:
            return yaml.safe_load(f) or {}

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        if self.camera is not None:
            frame = self.camera.undistort(frame)

        annotated, tags = detect_and_annotate(
            frame,
            self.detector,
            camera_params=self.camera.camera_params if self.camera else None,
            tag_size=0.05,
        )

        with self.lock:
            self.latest_frame = annotated.copy()
            self.latest_tags = tags
            self.image_count += 1

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="rgb8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if tags:
            import json
            detections = [
                {
                    "id": t.tag_id,
                    "corners": t.corners.tolist(),
                    "center": t.center.tolist() if hasattr(t, "center") else None,
                }
                for t in tags
            ]
            self.detections_pub.publish(String(data=json.dumps(detections)))

    def status_timer(self):
        with self.lock:
            tags = self.latest_tags
            count = self.image_count
            self.image_count = 0

        if count == 0:
            self.get_logger().warn("No new images received in the last second.")
            return

        if len(tags) == 0:
            self.get_logger().info(
                f"Receiving images ({count} fps), no AprilTags detected."
            )
        else:
            ids = [tag.tag_id for tag in tags]
            self.get_logger().info(
                f"Receiving images ({count} fps), "
                f"detected {len(ids)} tag(s): {ids}"
            )

    def get_frame(self):
        with self.lock:
            return (
                self.latest_frame.copy()
                if self.latest_frame is not None
                else None
            )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()

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
