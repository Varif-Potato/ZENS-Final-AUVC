import os
os.environ["OPENCV_OPENCL_RUNTIME"] = ""

import threading
from pathlib import Path
import cv2
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from cv.apriltag_detection import AprilTagDetector
from cv.camera import CameraCalibration
from cv.visualizer import detect_and_annotate, draw_yolo_detections
from cv.video_logger import VideoLogger
from cv.yolo_detection import YOLODetector

import ament_index_python


def _resolve_package_path(relative_path):
    share = ament_index_python.get_package_share_directory("cv")
    return os.path.join(share, relative_path)


class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_frame = None
        self.latest_tags = []
        self.image_count = 0

        config_dir = (
            self.declare_parameter("config_dir", "").value
            or _resolve_package_path("config")
        )

        # ---- AprilTag setup ----
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
        self.tag_size = detector_params.get("apriltag_detector", {}).get(
            "tag_size", 0.05
        )

        if os.path.exists(camera_info_path):
            self.camera = CameraCalibration.from_yaml(camera_info_path)
            self.get_logger().info(
                f"Loaded camera calibration from {camera_info_path}"
            )
        else:
            self.camera = None
            self.get_logger().warn(
                f"No camera calibration at {camera_info_path} — "
                "pose estimation disabled"
            )

        # ---- YOLO setup ----
        yolo_conf = detector_params.get("yolo_detector", {})
        self.yolo_enabled = yolo_conf.get("enabled", False)
        self.yolo = None

        if self.yolo_enabled:
            model_rel = yolo_conf.get("model_path", "models/auv_detector.onnx")
            model_path = str(Path(config_dir).parent / model_rel)
            labels_path = yolo_conf.get(
                "labels_path", "config/auv_labels.yaml"
            )
            labels_abs = os.path.join(config_dir, os.path.basename(labels_path))

            if os.path.exists(model_path):
                class_names = self._load_labels(labels_abs)
                self.yolo = YOLODetector(
                    model_path=model_path,
                    class_names=class_names,
                    conf_threshold=yolo_conf.get("conf_threshold", 0.25),
                    iou_threshold=yolo_conf.get("iou_threshold", 0.45),
                    input_size=(
                        yolo_conf.get("input_width", 416),
                        yolo_conf.get("input_height", 416),
                    ),
                )
                self.get_logger().info(
                    f"YOLO enabled, model loaded from {model_path}"
                )
            else:
                self.get_logger().warn(
                    f"YOLO enabled but model not found at {model_path} — "
                    "running AprilTag only"
                )
                self.yolo_enabled = False
        else:
            self.get_logger().info("YOLO disabled — AprilTag only")

        # ---- Topics ----
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
        self.yolo_pub = self.create_publisher(
            String, "/yolo/detections", 10
        )

        self.get_logger().info(
            f"Subscribed to '{camera_topic}', "
            f"publishing annotated -> /apriltags/annotated, "
            f"detections -> /apriltags/detections, "
            f"yolo -> /yolo/detections"
        )

        self.create_timer(1.0, self.status_timer)

    # -----------------------------------------------------------------

    def _load_yaml(self, path):
        import yaml
        with open(path) as f:
            return yaml.safe_load(f) or {}

    def _load_labels(self, path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("names", ["auv"])

    # -----------------------------------------------------------------

    def callback(self, msg):
        t0 = time.time()

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t1 = time.time()

        if self.camera is not None:
            frame = self.camera.undistort(frame)
        t2 = time.time()

        # --- AprilTag detection ---
        annotated, tags = detect_and_annotate(
            frame,
            self.detector,
            camera_params=self.camera.camera_params if self.camera else None,
            tag_size=self.tag_size,
        )
        t3 = time.time()

        # --- YOLO detection (on the same frame) ---
        yolo_results = []
        if self.yolo_enabled and self.yolo is not None:
            yolo_results = self.yolo.detect(frame)
            annotated = draw_yolo_detections(annotated, yolo_results)
        t4 = time.time()

        # --- Save latest ---
        with self.lock:
            self.latest_frame = annotated.copy()
            self.latest_tags = tags
            self.image_count += 1

        # --- Publish annotated image ---
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="rgb8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # --- Publish AprilTag detections (JSON) ---
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

        # --- Publish YOLO detections (JSON) ---
        if yolo_results:
            import json
            self.yolo_pub.publish(String(data=json.dumps(yolo_results)))

        t5 = time.time()
        self.get_logger().info(
            f"bridge={t1-t0:.3f}s undistort={t2-t1:.3f}s "
            f"apriltag={t3-t2:.3f}s yolo={t4-t3:.3f}s "
            f"publish={t5-t4:.3f}s"
        )

    # -----------------------------------------------------------------

    def status_timer(self):
        with self.lock:
            tags = self.latest_tags
            count = self.image_count
            self.image_count = 0

        if count == 0:
            self.get_logger().warn("No new images received in the last second.")
            return

        parts = [f"Receiving images ({count} fps)"]

        if len(tags) == 0:
            parts.append("no AprilTags detected")
        else:
            ids = [tag.tag_id for tag in tags]
            parts.append(f"AprilTags: {ids}")

        if self.yolo_enabled:
            parts.append("YOLO: active")
        else:
            parts.append("YOLO: disabled")

        self.get_logger().info(", ".join(parts))

    # -----------------------------------------------------------------

    def get_frame(self):
        with self.lock:
            return (
                self.latest_frame.copy()
                if self.latest_frame is not None
                else None
            )


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

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