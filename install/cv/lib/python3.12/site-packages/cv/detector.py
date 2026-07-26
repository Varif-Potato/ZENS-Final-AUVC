from dt_apriltags import Detector

class AprilTagDetector:
    def __init__(self, families="tag36h11", nthreads=1, quad_decimate=1.0,
                 quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0):
        self._detector = Detector(
            families=families,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=quad_sigma,
            refine_edges=refine_edges,
            decode_sharpening=decode_sharpening,
            debug=debug,
        )

    def detect(self, gray, estimate_tag_pose=False, camera_params=None, tag_size=None):
        return self._detector.detect(
            gray,
            estimate_tag_pose=estimate_tag_pose,
            camera_params=camera_params,
            tag_size=tag_size,
        )

    @classmethod
    def from_params(cls, params):
        return cls(
            families=params.get("families", "tag36h11"),
            nthreads=params.get("nthreads", 1),
            quad_decimate=params.get("quad_decimate", 1.0),
            quad_sigma=params.get("quad_sigma", 0.0),
            refine_edges=params.get("refine_edges", 1),
            decode_sharpening=params.get("decode_sharpening", 0.25),
            debug=params.get("debug", 0),
        )
