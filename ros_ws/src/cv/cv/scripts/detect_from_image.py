import sys
from pathlib import Path

import cv2

from cv.detector import AprilTagDetector
from cv.camera import CameraCalibration


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Detect AprilTags in a static image")
    parser.add_argument("image", help="Path to input image")
    parser.add_argument("-c", "--camera-info", default=None,
                        help="Camera calibration YAML (enables pose estimation)")
    parser.add_argument("-o", "--output", default=None,
                        help="Save annotated image to path")
    parser.add_argument("--tag-size", type=float, default=0.05,
                        help="Tag size in meters (default: 0.05)")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        print(f"Could not load image: {args.image}")
        sys.exit(1)

    if args.camera_info:
        cal = CameraCalibration.from_yaml(args.camera_info)
        img = cal.undistort(img)
        camera_params = cal.camera_params
        print(f"Loaded camera calibration, tag_size={args.tag_size}")
    else:
        camera_params = None

    detector = AprilTagDetector()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(
        gray,
        estimate_tag_pose=camera_params is not None,
        camera_params=camera_params,
        tag_size=args.tag_size if camera_params else None,
    )

    from cv.visualizer import draw_tags
    annotated = draw_tags(img, tags)

    print(f"Detected {len(tags)} tag(s):")
    for tag in tags:
        print(f"  ID {tag.tag_id} — center: ({tag.center[0]:.1f}, {tag.center[1]:.1f})")

    if args.output:
        cv2.imwrite(args.output, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
        print(f"Annotated image saved to {args.output}")

    cv2.imshow("AprilTag Detection", cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
