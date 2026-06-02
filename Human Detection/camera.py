"""Threaded camera capture.

A background thread keeps grabbing frames so the main loop never blocks waiting on
the camera. We always hand back the newest frame (buffer size 1) to avoid lag
build-up. This is the one place a small class earns its keep — it has to hold the
capture handle and share the latest frame across two threads.
"""
import platform
import threading
import time

import cv2


class Camera:
    def __init__(self, index, width, height):
        self.capture = self._open(index, width, height)
        self.lock = threading.Lock()
        self.latest_frame = None
        self.running = True

        if self.capture.isOpened():
            time.sleep(1.0)                       # let the sensor auto-expose
            _, self.latest_frame = self.capture.read()

        threading.Thread(target=self._grab_loop, daemon=True).start()

    def _open(self, index, width, height):
        """Open the camera with the right backend for the OS."""
        if platform.system() == "Windows":
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
            if not cap.isOpened():
                cap = cv2.VideoCapture(index)     # universal fallback
        else:
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)   # native Linux / Pi
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not cap.isOpened():
            print(f"[CAMERA] WARNING: could not open camera index {index}")
        return cap

    def _grab_loop(self):
        """Background thread: continuously store the most recent frame."""
        while self.running:
            ok, frame = self.capture.read()
            if ok:
                with self.lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.005)                 # camera hiccup, back off briefly

    def read(self):
        """Return a copy of the newest frame, or None if nothing has arrived yet."""
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def release(self):
        self.running = False
        time.sleep(0.1)
        if self.capture.isOpened():
            self.capture.release()
