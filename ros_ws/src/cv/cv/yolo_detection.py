import numpy as np
import cv2
import onnxruntime as ort
import time


class YOLODetector:
    def __init__(self, model_path, class_names, conf_threshold=0.25,
                 iou_threshold=0.45, input_size=(256, 256)):
        self.class_names = class_names
        self.num_classes = len(class_names)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.input_w, self.input_h = input_size

        providers = ["CPUExecutionProvider"]
        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 4
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, sess_options=sess_options, providers=providers)

        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        shape = inp.shape
        if isinstance(shape[2], int):
            self.input_h = shape[2]
            self.input_w = shape[3]
        else:
            self.input_h = self.input_h
            self.input_w = self.input_w

        self.output_names = [o.name for o in self.session.get_outputs()]

    def detect(self, frame):
        t0 = time.perf_counter()
        preprocessed, scale, pad_left, pad_top = self._preprocess(frame)
        t1 = time.perf_counter()
        outputs = self.session.run(
            self.output_names, {self.input_name: preprocessed}
        )
        t2 = time.perf_counter()
        result = self._postprocess(outputs, scale, pad_left, pad_top,
                                frame.shape[1], frame.shape[0])
        t3 = time.perf_counter()
        print(f"preprocess={t1-t0:.3f}s inference={t2-t1:.3f}s postprocess={t3-t2:.3f}s")
        return result

    def _preprocess(self, frame):
        h, w = frame.shape[:2]
        scale = min(self.input_w / w, self.input_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)

        resized = cv2.resize(frame, (new_w, new_h),
                             interpolation=cv2.INTER_LINEAR)

        dw = self.input_w - new_w
        dh = self.input_h - new_h
        left = dw // 2
        right = dw - left
        top = dh // 2
        bottom = dh - top

        padded = cv2.copyMakeBorder(
            resized, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )

        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        rgb = rgb.astype(np.float32) / 255.0
        nchw = np.transpose(rgb, (2, 0, 1))
        nchw = np.expand_dims(nchw, axis=0)
        return nchw, scale, left, top

    def _postprocess(self, outputs, scale, pad_left, pad_top,
                     orig_w, orig_h):
        pred = outputs[0]
        if pred.ndim == 3:
            pred = pred[0]
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T

        candidates = []
        for row in pred:
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])

            if confidence < self.conf_threshold:
                continue

            cx, cy, bw, bh = row[:4]

            x1 = (cx - bw / 2) - pad_left
            y1 = (cy - bh / 2) - pad_top
            x2 = (cx + bw / 2) - pad_left
            y2 = (cy + bh / 2) - pad_top

            x1 = int(max(0, x1 / scale))
            y1 = int(max(0, y1 / scale))
            x2 = int(min(orig_w, x2 / scale))
            y2 = int(min(orig_h, y2 / scale))

            candidates.append({
                "class_id": class_id,
                "class_name": self.class_names[class_id],
                "confidence": confidence,
                "bbox": [x1, y1, x2, y2],
            })

        if len(candidates) > 1:
            candidates = self._nms(candidates)

        return candidates

    def _nms(self, candidates):
        boxes = np.array([c["bbox"] for c in candidates], dtype=np.float32)
        scores = np.array([c["confidence"] for c in candidates],
                          dtype=np.float32)
        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(), scores.tolist(),
            self.conf_threshold, self.iou_threshold
        )
        if len(indices) == 0:
            return []
        return [candidates[i] for i in indices.flatten()]