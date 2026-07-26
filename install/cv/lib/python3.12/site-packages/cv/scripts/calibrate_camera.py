import sys
import glob
from pathlib import Path

import cv2
import numpy as np

from cv.camera import CameraCalibration


def calibrate(image_dir, pattern_size=(7, 6), square_size=0.025):
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []
    shape = None

    for fname in sorted(glob.glob(str(Path(image_dir) / "*.png"))):
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        shape = gray.shape[::-1]

        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not ret:
            print(f"Skipping {fname} — no corners found")
            continue

        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        objpoints.append(objp)
        imgpoints.append(corners2)

    if not objpoints:
        print("No valid calibration images found.")
        sys.exit(1)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, shape, None, None
    )

    cal = CameraCalibration.from_cv(mtx, dist, shape[0], shape[1])
    print(f"RMS re-projection error: {ret:.4f}")
    print(f"fx={cal.fx:.2f}, fy={cal.fy:.2f}, cx={cal.cx:.2f}, cy={cal.cy:.2f}")
    return cal


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Calibrate camera from chessboard images")
    parser.add_argument("image_dir", help="Directory containing calibration images (*.png)")
    parser.add_argument("-o", "--output", default="camera_info.yaml",
                        help="Output YAML path (default: camera_info.yaml)")
    parser.add_argument("--pattern-width", type=int, default=7,
                        help="Inner corners per row (default: 7)")
    parser.add_argument("--pattern-height", type=int, default=6,
                        help="Inner corners per column (default: 6)")
    parser.add_argument("--square-size", type=float, default=0.025,
                        help="Square size in meters (default: 0.025)")
    args = parser.parse_args()

    cal = calibrate(
        args.image_dir,
        pattern_size=(args.pattern_width, args.pattern_height),
        square_size=args.square_size,
    )
    cal.to_yaml(args.output)
    print(f"Calibration saved to {args.output}")


if __name__ == "__main__":
    main()
