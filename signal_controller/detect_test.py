r"""detect_test.py — run YOLO person detection on the drone's FORWARD camera. No flying.

Pulls the drone's RTSP video and runs person detection on it, drawing the boxes.

- If a DESKTOP is available (you have Remote Desktop) -> shows a LIVE window. Press 'q'
  to quit. RUN IT FROM A TERMINAL INSIDE THE REMOTE DESKTOP (not plain SSH), so it has
  a display.
- If run over plain SSH (no display) -> falls back to saving an annotated clip you can
  copy off and watch.

SETUP — the drone camera (192.168.1.1) collides with your home subnet, so force that
one address out wlan0 first:
    sudo ip route add 192.168.1.1/32 dev wlan0
    python3 detect_test.py [seconds]          # seconds only matters in save mode (default 20)
    sudo ip route del 192.168.1.1/32 dev wlan0   # undo after
"""
import os
import subprocess
import sys
import threading
import time

import cv2
import yaml
from ultralytics import YOLO

RTSP = "rtsp://192.168.1.1:7070/webcam"
CONF = 0.35
PERSON_CLASS = 0
DETECT_EVERY = 2           # run YOLO every Nth frame — frees CPU so the decoder isn't starved
OUT_VIDEO = "detect_out.mp4"
OUT_SNAP = "detect_snapshot.jpg"

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                 # LockeDrone/
LIVE = bool(os.environ.get("DISPLAY"))       # show a live window if a desktop is available

# The drone's RTSP is lossy over UDP. Tell ffmpeg to DISCARD corrupt frames instead of
# decoding them — a corrupt packet is what crashes the decoder ("malloc(): ... corrupted
# / Aborted"). Must be set before the VideoCapture is created.
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;udp|buffer_size;1048576|max_delay;500000|fflags;discardcorrupt"
)  # big receive buffer survives CPU-busy moments; discard corrupt frames


def load_model():
    """Load the model, preferring the fast ncnn export. Returns (model, imgsz).

    ncnn is much faster than the .pt on the Pi's ARM CPU, but it's FIXED-size: we read
    the resolution baked into the export (metadata.yaml) and run at exactly that, or
    ncnn throws 'malloc(): invalid size'. (Re-bake smaller for more speed via the
    "Human Detection/export_model.py".)
    """
    ncnn = os.path.join(ROOT, "yolo11n_ncnn_model")
    meta_path = os.path.join(ncnn, "metadata.yaml")
    if os.path.isfile(meta_path):               # the folder must actually contain the model
        size = yaml.safe_load(open(meta_path)).get("imgsz", 640)
        imgsz = size[0] if isinstance(size, (list, tuple)) else size
        print(f"[DET] ncnn model @ {imgsz}px (fast on ARM)")
        return YOLO(ncnn, task="detect"), imgsz
    print("[DET] ncnn model missing/incomplete — using yolo11n.pt (slower).")
    print("      For ncnn speed, regenerate it (on HOME wifi, for internet):")
    print("      cd ~/LockeDrone && python3 'Human Detection/export_model.py'")
    return YOLO(os.path.join(ROOT, "yolo11n.pt")), 192   # small input so the .pt isn't unbearable


def ensure_route_and_reachable():
    """Force the drone's IP out wlan0 (home/drone subnet collision) and confirm it answers.

    Needs root for the route add — you're running as root, so this just works and saves
    you the manual 'ip route add' every time.
    """
    subprocess.run(["ip", "route", "add", "192.168.1.1/32", "dev", "wlan0"],
                   capture_output=True)                 # "File exists" if already set = fine
    r = subprocess.run(["ping", "-I", "wlan0", "-c", "1", "-W", "2", "192.168.1.1"],
                       capture_output=True)
    return r.returncode == 0


class LatestFrame:
    """Background grabber that always holds the NEWEST frame, dropping the backlog.

    RTSP buffers frames; if YOLO is slower than the stream, that buffer fills and you
    watch seconds-old video (the "lag"). This thread reads + discards continuously so we
    always process the latest frame: low FPS, but live, not delayed.
    """
    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if ok:
                self.frame = f

    def read(self):
        return self.frame

    def stop(self):
        self.running = False


def annotate(frame, boxes):
    n = 0
    for box in boxes:
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        c = float(box.conf[0])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"person {c:.2f}", (x1, max(y1 - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        n += 1
    cv2.putText(frame, f"people: {n}", (8, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return n


def main():
    seconds = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    model, imgsz = load_model()

    print("[DET] setting drone route + checking reachability...")
    if not ensure_route_and_reachable():
        print("[DET] can't reach the drone at 192.168.1.1 via wlan0.")
        print("      Check: drone ON?  on FLOW-UFO wifi (run: iwgetid)?  phone disconnected?")
        return

    print(f"[DET] opening {RTSP} ...")
    cap = cv2.VideoCapture(RTSP, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[DET] could not open the stream.")
        print("      Add the route first:  sudo ip route add 192.168.1.1/32 dev wlan0")
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 352
    writer = None
    if LIVE:
        print(f"[DET] LIVE window ({w}x{h}) — press 'q' to quit.")
    else:
        writer = cv2.VideoWriter(OUT_VIDEO, cv2.VideoWriter_fourcc(*"mp4v"), 15, (w, h))
        print(f"[DET] no display — saving {seconds}s to {OUT_VIDEO} (run from the desktop for a live window).")

    grabber = LatestFrame(cap)
    t0 = time.time()
    frames = 0
    last = None
    last_boxes = []
    try:
        while True:
            frame = grabber.read()
            if frame is None:
                time.sleep(0.01)
                continue
            frames += 1
            # only run YOLO every Nth frame; reuse the last boxes in between (saves CPU)
            if frames % DETECT_EVERY == 0:
                results = model(frame, conf=CONF, classes=[PERSON_CLASS], imgsz=imgsz, verbose=False)
                last_boxes = results[0].boxes
            annotate(frame, last_boxes)
            last = frame

            if LIVE:
                cv2.imshow("Drone Detection (press q to quit)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                writer.write(frame)
                if frames % 15 == 0:
                    print(f"  ...{frames} frames")
                if time.time() - t0 >= seconds:
                    break
    except KeyboardInterrupt:
        print("\n[DET] stopped.")
    finally:
        grabber.stop()
        if writer:
            writer.release()
        cap.release()
        cv2.destroyAllWindows()

    fps = frames / (time.time() - t0) if frames else 0
    if last is not None:
        cv2.imwrite(OUT_SNAP, last)
    print(f"[DET] {frames} frames, ~{fps:.1f} FPS. Saved {OUT_SNAP}"
          + ("" if LIVE else f" and {OUT_VIDEO}") + ".")


if __name__ == "__main__":
    main()
