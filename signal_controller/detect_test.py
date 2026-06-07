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
import sys
import time

import cv2
from ultralytics import YOLO

RTSP = "rtsp://192.168.1.1:7070/webcam"
CONF = 0.35
PERSON_CLASS = 0
IMGSZ = 320                 # smaller = faster on the Pi
OUT_VIDEO = "detect_out.mp4"
OUT_SNAP = "detect_snapshot.jpg"

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                 # LockeDrone/
LIVE = bool(os.environ.get("DISPLAY"))       # show a live window if a desktop is available


def find_model():
    for cand in (os.path.join(ROOT, "yolo11n.pt"),
                 os.path.join(ROOT, "yolo11n_ncnn_model"),
                 "yolo11n.pt"):
        if os.path.exists(cand):
            return cand
    return "yolo11n.pt"


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
    model_path = find_model()
    print(f"[DET] loading model: {model_path}")
    model = YOLO(model_path, task="detect") if model_path.endswith("_ncnn_model") else YOLO(model_path)

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

    t0 = time.time()
    frames = 0
    last = None
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            results = model(frame, conf=CONF, classes=[PERSON_CLASS], imgsz=IMGSZ, verbose=False)
            annotate(frame, results[0].boxes)
            last = frame
            frames += 1

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
