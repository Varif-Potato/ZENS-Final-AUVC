import cv2

def draw_tags(frame, tags):
    color_img = cv2.cvtColor(
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2RGB
    )
    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(
                color_img,
                tuple(tag.corners[idx - 1, :].astype(int)),
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0),
            )
        cv2.putText(
            color_img,
            str(tag.tag_id),
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
