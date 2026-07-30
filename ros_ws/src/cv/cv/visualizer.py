import cv2
import numpy as np


_COLORS = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
    (0, 0, 128), (128, 128, 0), (128, 0, 128), (0, 128, 128),
]


def draw_tags(frame, tags):
    color_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(
                color_img,
                tuple(tag.corners[idx - 1, :].astype(int)),
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0),
            )
        label = str(tag.tag_id)
        if hasattr(tag, 'pose_t') and tag.pose_t is not None:
            dist = float(np.linalg.norm(tag.pose_t))
            label = f"{tag.tag_id} {dist:.2f}m"
        cv2.putText(
            color_img,
            label,
            org=(
                tag.corners[0, 0].astype(int) + 10,
                tag.corners[0, 1].astype(int) + 10,
            ),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255),
        )
    return color_img


def detect_and_annotate(frame, at_detector, camera_params=None, tag_size=None):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(
        gray,
        estimate_tag_pose=camera_params is not None,
        camera_params=camera_params,
        tag_size=tag_size,
    )
    annotated = draw_tags(frame, tags)
    return annotated, tags


def draw_yolo_detections(frame, detections, conf_threshold=0.25):
    color_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    for det in detections:
        if det["confidence"] < conf_threshold:
            continue
        x1, y1, x2, y2 = det["bbox"]
        color = _COLORS[det["class_id"] % len(_COLORS)]
        cv2.rectangle(color_img, (x1, y1), (x2, y2), color, 2)
        label = f"{det['class_name']} {det['confidence']:.2f}"
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )
        cv2.rectangle(
            color_img, (x1, y1 - th - 4), (x1 + tw + 4, y1), color, -1
        )
        cv2.putText(
            color_img, label, (x1 + 2, y1 - 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )
    return color_img