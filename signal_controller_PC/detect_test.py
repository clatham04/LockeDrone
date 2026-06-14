r"""detect_test.py — run YOLO person detection on the drone's FORWARD camera. No flying.

Pulls the drone's RTSP video (via DroneCamera — an ffmpeg subprocess that survives the
lossy stream) and runs person detection on it, drawing the boxes.

- If a DESKTOP is available (you have Remote Desktop) -> shows a LIVE window. Press 'q'
  to quit. RUN IT FROM A TERMINAL INSIDE THE REMOTE DESKTOP (not plain SSH).
- If run over plain SSH (no display) -> saves an annotated clip you can copy off.

The route to the drone (192.168.1.1, which collides with your home subnet) is added
automatically — run as Administrator. Needs the ffmpeg CLI.

    python detect_test.py [seconds]      # seconds only matters in save mode (default 20)
"""
import os
import subprocess
import sys
import time

import cv2
import yaml
from ultralytics import YOLO

from drone_camera import DroneCamera

RTSP = "rtsp://192.168.1.1:7070/webcam"
CONF = 0.35
PERSON_CLASS = 0
DETECT_EVERY = 2           # run YOLO every Nth frame — frees CPU so the decoder isn't starved
OUT_VIDEO = "detect_out.mp4"
OUT_SNAP = "detect_snapshot.jpg"

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                 # LockeDrone/
LIVE = True  # Windows always has a display; set to False if you want to save a clip instead


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
    return YOLO(os.path.join(ROOT, "yolo11n.pt")), 192   # small input so the .pt isn't unbearable


def ensure_route_and_reachable():
    """Force the drone's IP out the correct interface and confirm it answers."""
    # Add a host route to the drone (Windows equivalent of 'ip route add ... dev wlan0')
    subprocess.run(
        ["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
        capture_output=True
    )  # ignore errors — "Element exists" is fine

    # Windows ping: -n 1 (1 packet), -w 2000 (2s timeout)
    r = subprocess.run(
        ["ping", "-n", "1", "-w", "2000", "192.168.1.1"],
        capture_output=True
    )
    return r.returncode == 0


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
        print("[DET] can't reach the drone at 192.168.1.1.")
        print("      Check: drone ON?  connected to FLOW-UFO wifi?  phone disconnected?")
        print("      Also make sure you're running as Administrator.")
        return

    print(f"[DET] opening {RTSP} (ffmpeg subprocess — survives the lossy stream) ...")
    cam = DroneCamera(RTSP, debug=True)            # uses default 640x352, shows ffmpeg errors

    writer = None
    if LIVE:
        print(f"[DET] LIVE window ({cam.w}x{cam.h}) — press 'q' to quit.")
    else:
        writer = cv2.VideoWriter(OUT_VIDEO, cv2.VideoWriter_fourcc(*"mp4v"), 15, (cam.w, cam.h))
        print(f"[DET] no display — saving {seconds}s to {OUT_VIDEO} (set LIVE=True for a live window).")

    # wait for the first frame (ffmpeg connect + first decode can take a couple seconds)
    print("[DET] waiting for first frame...")
    t_wait = time.time()
    while cam.read() is None:
        if time.time() - t_wait > 15:
            print("[DET] no video after 15s — is the drone streaming? (check it's ON + on FLOW-UFO)")
            cam.stop()
            return
        time.sleep(0.1)

    t0 = time.time()
    frames = 0
    last = None
    last_boxes = []
    try:
        while True:
            frame = cam.read()
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
        cam.stop()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    fps = frames / (time.time() - t0) if frames else 0
    if last is not None:
        cv2.imwrite(OUT_SNAP, last)
    print(f"[DET] {frames} frames, ~{fps:.1f} FPS. Saved {OUT_SNAP}"
          + ("" if LIVE else f" and {OUT_VIDEO}") + ".")


if __name__ == "__main__":
    main()