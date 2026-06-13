r"""drone_camera.py — robust RTSP capture for the drone's lossy video.

The drone's RTSP stream drops packets, and OpenCV's in-process decoder crashes
NATIVELY on the corruption (malloc/segfault — uncatchable in Python). This decodes
via an ffmpeg SUBPROCESS instead: a decode crash kills only that subprocess, which we
auto-restart, so the caller never dies. A background thread keeps the newest frame
(low lag), and read() never blocks.

Needs the ffmpeg CLI: https://ffmpeg.org/download.html (add bin folder to PATH)

    cam = DroneCamera("rtsp://192.168.1.1:7070/webcam")
    frame = cam.read()      # newest BGR frame, or None until the first one arrives
    cam.stop()
"""
import re
import shutil
import subprocess
import threading
import time

import numpy as np

# udp transport (the drone only supports udp). Do NOT discard corrupt frames — over this
# lossy link the keyframes arrive slightly corrupt, and discarding them means ffmpeg never
# gets a frame to start from. We tolerate glitches instead; the subprocess isolation (not
# discarding) is what protects us from a decoder crash.
FFMPEG_OPTS = ["-rtsp_transport", "udp", "-err_detect", "ignore_err"]


def ensure_route(url):
    """Pin the drone's IP via a host route (Windows). The home network and the drone's AP
    both use 192.168.1.x, so without this the camera packets go to your home router and the
    stream never opens. Run as Administrator. 'Element exists' = already set."""
    m = re.search(r"://([^:/]+)", url)
    if m:
        ip = m.group(1)
        subprocess.run(
            ["route", "add", ip, "mask", "255.255.255.255", ip],
            capture_output=True
        )  # ignore errors — already exists is fine


def probe_size(url, fallback=(640, 352), retries=3):
    """Ask ffprobe for the stream's actual WxH, with retries if it fails."""
    if not shutil.which("ffprobe"):
        print(f"[CAM] ffprobe not found — using fallback size {fallback[0]}x{fallback[1]}")
        return fallback
    for attempt in range(retries):
        try:
            out = subprocess.run(
                ["ffprobe", "-v", "error", "-rtsp_transport", "udp", "-select_streams", "v:0",
                 "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", url],
                capture_output=True, text=True, timeout=12).stdout.strip()
            w, h = (int(x) for x in out.split("x")[:2])
            if w > 0 and h > 0:
                return w, h
            print(f"[CAM] probe returned 0x0 (attempt {attempt+1}/{retries}), retrying...")
        except Exception as e:
            print(f"[CAM] probe failed (attempt {attempt+1}/{retries}): {e}")
        time.sleep(2)
    print(f"[CAM] probe failed after {retries} attempts — using fallback {fallback[0]}x{fallback[1]}")
    return fallback


class DroneCamera:
    def __init__(self, url, width=640, height=352, debug=False):
        if not shutil.which("ffmpeg"):
            raise RuntimeError(
                "ffmpeg CLI not found. Download it from https://ffmpeg.org/download.html "
                "and add the bin folder to your system PATH."
            )
        ensure_route(url)
        self.url = url
        self.debug = debug
        self.w, self.h = width, height
        print(f"[CAM] stream size: {self.w}x{self.h}")
        self.frame = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _spawn(self):
        cmd = ["ffmpeg", "-nostdin", "-loglevel", "error", *FFMPEG_OPTS,
               "-i", self.url, "-an", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"]
        stderr = None if self.debug else subprocess.DEVNULL   # debug=True -> see ffmpeg errors
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=stderr)

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
            except Exception as e:
                if self.debug:
                    print(f"[CAM] reader error: {e}")
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