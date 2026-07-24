import threading
import time
from datetime import datetime
import cv2

class VideoLogger:
    def __init__(self, get_frame, output_path=None, fps=15.0):
        self.get_frame = get_frame
        self.fps = fps
        self.running = False
        self.thread = None
        self.writer = None
        self.output_path = output_path or f"apriltag_log_{datetime.now():%Y%m%d_%H%M%S}.mp4"

    def _loop(self):
        interval = 1.0 / self.fps
        while self.running:
            frame = self.get_frame()
            if frame is not None:
                if self.writer is None:
                    h, w = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    self.writer = cv2.VideoWriter(
                        self.output_path, fourcc, self.fps, (w, h)
                    )
                self.writer.write(frame)
            time.sleep(interval)

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)
        if self.writer is not None:
            self.writer.release()
            self.writer = None
