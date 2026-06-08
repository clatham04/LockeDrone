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


def ensure_route(url, iface="wlan0"):
    """Pin the drone's IP to wlan0. The home network and the drone's AP both use
    192.168.1.x, so without this the camera packets go to your home router and the
    stream never opens. Needs root (you run with sudo). 'File exists' = already set."""
    m = re.search(r"://([^:/]+)", url)
    if m:
        subprocess.run(["ip", "route", "add", f"{m.group(1)}/32", "dev", iface],
                       capture_output=True)


def probe_size(url, fallback=(640, 352)):
    """Ask ffprobe for the stream's actual WxH (also a quick connection test)."""
    if not shutil.which("ffprobe"):
        return fallback
    try:
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-rtsp_transport", "udp", "-select_streams", "v:0",
             "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", url],
            capture_output=True, text=True, timeout=12).stdout.strip()
        w, h = (int(x) for x in out.split("x")[:2])
        return w, h
    except Exception:
        return fallback


class DroneCamera:
    def __init__(self, url, width=None, height=None, debug=False):
        if not shutil.which("ffmpeg"):
            raise RuntimeError("ffmpeg CLI not found. Install it:  sudo apt install -y ffmpeg")
        ensure_route(url)                          # fix the home/drone subnet collision first
        self.url = url
        self.debug = debug
        if width and height:
            self.w, self.h = width, height
        else:
            self.w, self.h = probe_size(url)
            print(f"[CAM] detected stream size: {self.w}x{self.h}")
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
