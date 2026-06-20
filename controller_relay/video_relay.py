r"""video_relay.py — the VIDEO path:  drone (wlan0) ──► Pi ──► desktop (eth0).

Pulls the drone's H.264 RTSP on wlan0 and forwards the H.264 bitstream to the desktop over
eth0 as RTP/UDP, with **NO re-encode** (lowest latency). The Pi never decodes a frame.

GStreamer passthrough pipeline:
    rtspsrc (drone)  ->  rtph264depay  ->  h264parse  ->  rtph264pay  ->  udpsink (desktop)
  - depay/parse/pay just RE-PACKETIZE the H.264; there is NO decoder/encoder in the path.
  - config-interval=1 re-sends SPS/PPS so the desktop can start decoding mid-stream.
  - If the drone camera ever turns out to be MJPEG/raw instead of H.264, THIS is the only
    file that changes (insert `! v4l2h264enc` / the Pi's hardware encoder before the pay).

Auto-restarts the pipeline if the lossy stream stalls it.
"""
import shutil
import subprocess
import threading
import time


class VideoRelay:
    def __init__(self, cfg):
        self.rtsp = cfg["drone"]["rtsp_url"]
        self.host = cfg["desktop"]["ip"]
        self.port = cfg["desktop"]["video_port"]
        self._proc = None
        self._running = False

    def _pipeline(self):
        return [
            "gst-launch-1.0", "-q",
            "rtspsrc", f"location={self.rtsp}", "protocols=udp", "latency=0", "!",
            "rtph264depay", "!",
            "h264parse", "config-interval=1", "!",
            "rtph264pay", "pt=96", "config-interval=1", "!",
            "udpsink", f"host={self.host}", f"port={self.port}", "sync=false", "async=false",
        ]

    def start(self):
        if not shutil.which("gst-launch-1.0"):
            raise RuntimeError(
                "GStreamer not found. Install on the Pi:\n"
                "  sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base "
                "gstreamer1.0-plugins-good gstreamer1.0-plugins-bad"
            )
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        print(f"[VIDEO] {self.rtsp}  ->  {self.host}:{self.port}  (H.264 passthrough, RTP/UDP, no re-encode)")

    def _loop(self):
        while self._running:
            self._proc = subprocess.Popen(self._pipeline())
            self._proc.wait()                      # pipeline died (stream blip) -> restart
            if self._running:
                time.sleep(0.5)

    def stop(self):
        self._running = False
        if self._proc:
            try:
                self._proc.terminate()
            except Exception:
                pass
