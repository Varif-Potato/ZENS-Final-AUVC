import json
from pathlib import Path
import yaml
import numpy as np
import cv2

class CameraCalibration:
    def __init__(self, fx=0.0, fy=0.0, cx=0.0, cy=0.0,
                 k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0,
                 width=640, height=480):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.k1 = k1
        self.k2 = k2
        self.p1 = p1
        self.p2 = p2
        self.k3 = k3
        self.width = width
        self.height = height

    @property
    def camera_matrix(self):
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1],
        ], dtype=np.float64)

    @property
    def dist_coeffs(self):
        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3],
                        dtype=np.float64)

    @property
    def camera_params(self):
        return (self.fx, self.fy, self.cx, self.cy)

    def to_yaml(self, path):
        data = {
            "image_width": self.width,
            "image_height": self.height,
            "camera_matrix": {
                "fx": self.fx, "fy": self.fy,
                "cx": self.cx, "cy": self.cy,
            },
            "distortion_coefficients": {
                "k1": self.k1, "k2": self.k2,
                "p1": self.p1, "p2": self.p2, "k3": self.k3,
            },
        }
        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False)

    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            data = yaml.safe_load(f)
        cm = data.get("camera_matrix", {})
        dc = data.get("distortion_coefficients", {})
        return cls(
            width=data.get("image_width", 640),
            height=data.get("image_height", 480),
            fx=cm.get("fx", 0.0), fy=cm.get("fy", 0.0),
            cx=cm.get("cx", 0.0), cy=cm.get("cy", 0.0),
            k1=dc.get("k1", 0.0), k2=dc.get("k2", 0.0),
            p1=dc.get("p1", 0.0), p2=dc.get("p2", 0.0),
            k3=dc.get("k3", 0.0),
        )

    @classmethod
    def from_cv(cls, mtx, dist, width, height):
        return cls(
            width=width, height=height,
            fx=mtx[0, 0], fy=mtx[1, 1],
            cx=mtx[0, 2], cy=mtx[1, 2],
            k1=dist[0, 0], k2=dist[0, 1],
            p1=dist[0, 2], p2=dist[0, 3],
            k3=dist[0, 4] if dist.shape[1] >= 5 else 0.0,
        )

    def to_json(self):
        return json.dumps({
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "width": self.width,
            "height": self.height,
        })

    def undistort(self, frame):
        h, w = frame.shape[:2]
        new_K, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, new_K)
