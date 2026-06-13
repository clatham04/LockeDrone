r"""follow_human — the drone follows a person, vision-driven and flight-safe.

It watches the forward camera, finds the (largest/closest) person, and turns their
bounding box into the four sticks. Goals:

  - NEVER run into the human. Hold a follow distance and back off if the box gets
    too big or the whole body no longer fits the frame (that means too close).
  - Level with the head. Raise/lower so the head (top of the box) sits at a target
    height in the frame.
  - Follow where they go. YAW turns to keep them centred; PITCH moves in/out to hold
    distance. Walk away -> box shrinks -> move closer. Stand still -> hover.
  - Stay low + reachable. Altitude pushes are small and bounded.
  - Search when lost. No person -> spin slowly IN PLACE (yaw only, no drift) until
    someone comes into view, then lock on. Never flies off.

Detection (YOLO, ~8 FPS) runs in its OWN thread; the 25 Hz control loop just reads
the latest box, so flight stays smooth and a video glitch simply -> hover.

Tuning lives in config.json under tuning.follow. If a stick drives the WRONG way
during your tied-string test, flip the matching *_sign (1 / -1) in config.
"""
import os
import subprocess
import threading
import time

import cv2
import yaml
from ultralytics import YOLO

from drone_camera import DroneCamera

RTSP = "rtsp://192.168.1.1:7070/webcam"
PERSON_CLASS = 0
CENTER = 128

HERE = os.path.dirname(os.path.abspath(__file__))     # .../behaviors
ROOT = os.path.dirname(os.path.dirname(HERE))         # project root

DEFAULTS = {
    "conf": 0.20,
    "target_dist_h": 0.60,
    "too_close_h": 0.78,
    "target_head_y": 0.28,
    "edge_margin": 0.04,
    "deadzone": 0.05,
    "yaw_gain": 110.0,
    "pitch_gain": 160.0,
    "throttle_gain": 130.0,
    "max_yaw": 45,
    "max_pitch": 32,
    "max_throttle": 24,
    "yaw_sign": 1,
    "pitch_sign": 1,
    "throttle_sign": 1,
    "stale_s": 0.8,
    "search_yaw": 18,
}

_cfg = dict(DEFAULTS)
_cam = None
_model = None
_imgsz = 256
_running = False
_lock = threading.Lock()
_latest = None


def _ensure_route():
    """Pin the drone's IP via a host route on Windows."""
    subprocess.run(
        ["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
        capture_output=True
    )


def _load_model():
    """Prefer the fast ncnn export; fall back to the .pt."""
    ncnn = os.path.join(ROOT, "yolo11n_ncnn_model")
    meta = os.path.join(ncnn, "metadata.yaml")
    if os.path.isfile(meta):
        size = yaml.safe_load(open(meta)).get("imgsz", 640)
        imgsz = size[0] if isinstance(size, (list, tuple)) else size
        return YOLO(ncnn, task="detect"), imgsz
    return YOLO(os.path.join(ROOT, "yolo11n.pt")), 256


def _biggest_person(boxes, fw, fh):
    """Largest person box -> frame-fraction summary, or None."""
    best, best_area = None, 0.0
    for b in boxes:
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
        area = (x2 - x1) * (y2 - y1)
        if area > best_area:
            best, best_area = (x1, y1, x2, y2), area
    if best is None:
        return None
    x1, y1, x2, y2 = best
    return {"cx": (x1 + x2) / 2 / fw, "top": y1 / fh,
            "bottom": y2 / fh, "h": (y2 - y1) / fh, "ts": time.time()}


def _detect_loop():
    """Continuously detect the person, publish the latest box, and show a live window."""
    global _latest
    conf = _cfg["conf"]
    _dbg_count = 0
    while _running:
        frame = _cam.read()
        if frame is None:
            time.sleep(0.02)
            continue
        fh, fw = frame.shape[:2]
        results = _model(frame, conf=conf, classes=[PERSON_CLASS], imgsz=_imgsz, verbose=False)
        det = _biggest_person(results[0].boxes, fw, fh)
        with _lock:
            _latest = det

        # draw detection box and head tracking dot on frame
        display = frame.copy()
        fh_disp, fw_disp = display.shape[:2]

        if det:
            # full body box
            x1 = int((det["cx"] - det["h"] * 0.3) * fw_disp)
            x2 = int((det["cx"] + det["h"] * 0.3) * fw_disp)
            y1 = int(det["top"] * fh_disp)
            y2 = int(det["bottom"] * fh_disp)
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # head center dot — top 15% of the box
            head_x = int(det["cx"] * fw_disp)
            head_y = int((det["top"] + det["h"] * 0.10) * fh_disp)
            cv2.circle(display, (head_x, head_y), 8, (0, 0, 255), -1)   # red filled dot
            cv2.circle(display, (head_x, head_y), 12, (255, 255, 255), 2)  # white ring

            # crosshair lines from head dot
            cv2.line(display, (head_x - 20, head_y), (head_x + 20, head_y), (0, 0, 255), 1)
            cv2.line(display, (head_x, head_y - 20), (head_x, head_y + 20), (0, 0, 255), 1)

            # target head position line (where we want the head to be)
            target_y = int(_cfg["target_head_y"] * fh_disp)
            cv2.line(display, (0, target_y), (fw_disp, target_y), (0, 255, 255), 1)

            cv2.putText(display, f"h:{det['h']:.2f} cx:{det['cx']:.2f}",
                        (x1, max(y1 - 8, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            status = "TRACKING"
            color = (0, 255, 0)
        else:
            # show center crosshair when searching
            cx, cy = fw_disp // 2, fh_disp // 2
            cv2.line(display, (cx - 30, cy), (cx + 30, cy), (0, 165, 255), 2)
            cv2.line(display, (cx, cy - 30), (cx, cy + 30), (0, 165, 255), 2)
            status = "SEARCHING..."
            color = (0, 165, 255)

        cv2.putText(display, status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow("Drone Camera — press ESC to close", display)
        cv2.waitKey(1)

        # print detection status every 2 seconds
        _dbg_count += 1
        if _dbg_count % 16 == 0:
            if det:
                print(f"[FOLLOW] person detected — cx:{det['cx']:.2f} h:{det['h']:.2f} top:{det['top']:.2f}")
            else:
                print("[FOLLOW] no person detected — searching...")


def start(state, config):
    global _cfg, _cam, _model, _imgsz, _running
    _cfg = {**DEFAULTS, **config.get("tuning", {}).get("follow", {})}
    _ensure_route()
    print("[FOLLOW] starting camera + detector...")
    _cam = DroneCamera(RTSP)
    _model, _imgsz = _load_model()
    print(f"[FOLLOW] model ready @ {_imgsz}px — searching for a person...")
    print("[FOLLOW] spinning to search. Will lock on when someone comes into view.")
    _running = True
    threading.Thread(target=_detect_loop, daemon=True).start()


def stop(state, config):
    global _running
    _running = False
    if _cam:
        _cam.stop()
    cv2.destroyAllWindows()
    print("[FOLLOW] stopped.")


def _stick(deviation, limit):
    deviation = max(-limit, min(limit, deviation))
    return int(CENTER + deviation)


def _gated(error, gain, deadzone):
    return 0.0 if abs(error) < deadzone else gain * error


def controller(state):
    """Read the latest detection and return (roll, pitch, throttle, yaw). 128 = hold."""
    with _lock:
        det = _latest

    # No person found / stale -> spin slowly in place to search
    if det is None or (time.time() - det["ts"]) > _cfg["stale_s"]:
        return CENTER, CENTER, CENTER, _stick(_cfg["search_yaw"], _cfg["max_yaw"])

    dz = _cfg["deadzone"]

    # YAW: turn to keep the person centred left/right
    yaw_dev = _gated(det["cx"] - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]

    # PITCH: hold follow distance, never collide
    body_cut = det["top"] < _cfg["edge_margin"] and det["bottom"] > (1 - _cfg["edge_margin"])
    if det["h"] > _cfg["too_close_h"] or body_cut:
        pitch_dev = -_cfg["max_pitch"]             # too close -> full back-off
    else:
        pitch_dev = _gated(_cfg["target_dist_h"] - det["h"], _cfg["pitch_gain"], dz) * _cfg["pitch_sign"]

    # THROTTLE: keep head at target height in frame
    thr_dev = -_gated(det["top"] - _cfg["target_head_y"], _cfg["throttle_gain"], dz) * _cfg["throttle_sign"]

    roll = CENTER
    pitch = _stick(pitch_dev, _cfg["max_pitch"])
    throttle = _stick(thr_dev, _cfg["max_throttle"])
    yaw = _stick(yaw_dev, _cfg["max_yaw"])

    # debug: print pitch every ~1 second so we can see what direction it's pushing
    if not hasattr(controller, "_dbg"):
        controller._dbg = 0
    controller._dbg += 1
    if controller._dbg % 25 == 0:
        direction = "FORWARD" if pitch > 128 else "BACKWARD" if pitch < 128 else "HOLD"
        print(f"[FOLLOW] pitch={pitch} ({direction})  h={det['h']:.2f}  target={_cfg['target_dist_h']}")

    return roll, pitch, throttle, yaw