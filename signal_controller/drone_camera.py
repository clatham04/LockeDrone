r"""drone_camera.py — robust RTSP capture for the drone's lossy video.

The drone's RTSP stream drops packets, and OpenCV's in-process decoder crashes
NATIVELY on the corruption (malloc/segfault — uncatchable in Python). This decodes
via an ffmpeg SUBPROCESS instead: a decode crash kills only that subprocess, which we
auto-restart, so the caller never dies. A background thread keeps the newest frame
(low lag), and read() never blocks.

Needs the ffmpeg CLI:  sudo apt install -y ffmpeg

    cam = DroneCamera("rtsp://192.168.1.1:7070/webcam")
    frame = cam.read()      # newest BGR frame, or None until the first one arrives
    cam.stop()
"""
import subprocess
import threading
import time

import numpy as np

# udp transport (the drone only supports udp), discard corrupt frames, tolerate errors.
FFMPEG_OPTS = ["-rtsp_transport", "udp", "-fflags", "discardcorrupt", "-err_detect", "ignore_err"]


class DroneCamera:
    def __init__(self, url, width=640, height=352):
        self.url = url
        self.w = width
        self.h = height
        self.frame = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _spawn(self):
        cmd = ["ffmpeg", "-nostdin", "-loglevel", "fatal", *FFMPEG_OPTS,
               "-i", self.url, "-an", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def _loop(self):
        n = self.w * self.h * 3
        while self.running:
            proc = self._spawn()
            try:
                while self.running:
                    raw = proc.stdout.read(n)
                    if not raw or len(raw) < n:
                        break                       # ffmpeg died/stalled -> restart it
                    self.frame = np.frombuffer(raw, np.uint8).reshape((self.h, self.w, 3))
            finally:
                try:
                    proc.kill()
                except Exception:
                    pass
            if self.running:
                time.sleep(0.5)                     # brief pause before respawning

    def read(self):
        """Newest frame (a copy), or None if none yet."""
        f = self.frame
        return None if f is None else f.copy()

    def stop(self):
        self.running = False
