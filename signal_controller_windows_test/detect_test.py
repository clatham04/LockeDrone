r"""detect_test.py (Windows) — YOLO person detection on the drone's forward camera.

Windows version of the detection preview: always shows a LIVE window (press 'q' to quit).
Uses a BIGGER model + full-resolution inference than the Pi (the PC has the headroom),
so it detects smaller / farther / partially-hidden people and runs every frame. Bump
MODEL to yolo11l.pt / yolo11x.pt for even more (best with a CUDA GPU).

Setup (see README.md in this folder):
  1. winget install Gyan.FFmpeg          (then open a NEW terminal)
  2. pip install -r requirements.txt
  3. connect your PC to the drone's wifi (FLOW-UFO) ONLY — disconnect home wifi/ethernet
  4. python detect_test.py
"""
import os
import subprocess
import time

import cv2
import torch
from ultralytics import YOLO

from drone_camera import DroneCamera

RTSP = "rtsp://192.168.1.1:7070/webcam"
CONF = 0.30                # a touch lower than the Pi -> catches more / farther people
PERSON_CLASS = 0
# Bigger model = better detection (smaller/farther/partially-hidden people). On the PC we
# have headroom the Pi doesn't. Accuracy/slowness order: n < s < m < l < x.
# Bump to "yolo11l.pt" or "yolo11x.pt" for max accuracy (best with a CUDA GPU).
MODEL = "yolo11m.pt"
IMGSZ = 640                # full-resolution inference -> more detail, sees more

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                 # LockeDrone/


def load_model():
    """Load the chosen model; ultralytics auto-downloads it (to the cwd) the first time."""
    dev = "GPU (CUDA)" if torch.cuda.is_available() else "CPU"
    print(f"[DET] model: {MODEL} @ {IMGSZ}px on {dev} (auto-downloads if missing)")
    return YOLO(MODEL)


def reachable():
    """Windows ping the drone so we fail fast with a clear message if it's not connected."""
    r = subprocess.run(["ping", "-n", "1", "-w", "2000", "192.168.1.1"], capture_output=True)
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
    model = load_model()

    print("[DET] checking the drone is reachable...")
    if not reachable():
        print("[DET] can't reach the drone at 192.168.1.1.")
        print("      Connect this PC to the FLOW-UFO wifi (and disconnect home wifi/ethernet,")
        print("      since both use 192.168.1.x). Make sure the drone is ON + phone disconnected.")
        return

    print(f"[DET] opening {RTSP} (ffmpeg subprocess — survives the lossy stream) ...")
    cam = DroneCamera(RTSP, debug=True)            # auto-detect size + show ffmpeg errors

    print("[DET] waiting for first frame...")
    t_wait = time.time()
    while cam.read() is None:
        if time.time() - t_wait > 15:
            print("[DET] no video after 15s — is the drone streaming? (ON + on FLOW-UFO)")
            cam.stop()
            return
        time.sleep(0.1)

    print(f"[DET] LIVE window ({cam.w}x{cam.h}) — press 'q' to quit.")
    t0 = time.time()
    frames = 0
    try:
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.01)
                continue
            results = model(frame, conf=CONF, classes=[PERSON_CLASS], imgsz=IMGSZ, verbose=False)
            annotate(frame, results[0].boxes)
            frames += 1
            cv2.imshow("Drone Detection (press q to quit)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\n[DET] stopped.")
    finally:
        cam.stop()
        cv2.destroyAllWindows()

    fps = frames / (time.time() - t0) if frames else 0
    print(f"[DET] {frames} frames, ~{fps:.1f} FPS.")


if __name__ == "__main__":
    main()
