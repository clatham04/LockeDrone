r"""follow_human — the drone follows a person, vision-driven and flight-safe.

Searches by spinning until a person is found, then tracks the largest person:
- YAW: keeps them centered left/right
- PITCH: holds follow distance (target_dist_h)
- THROTTLE: keeps head at target height in frame
- Detection is smoothed to prevent jumping between body parts
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
SMOOTH = 0.4   # exponential smoothing factor (0=no update, 1=no smoothing)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

DEFAULTS = {
    "conf": 0.20,
    "target_dist_h": 0.70,
    "too_close_h": 0.92,
    "edge_margin": 0.0,
    "target_head_y": 0.35,
    "deadzone": 0.05,
    "yaw_gain": 110.0,
    "pitch_gain": 280.0,
    "throttle_gain": 160.0,
    "max_yaw": 45,
    "max_pitch": 50,
    "max_throttle": 30,
    "yaw_sign": 1,
    "pitch_sign": 1,
    "throttle_sign": 1,
    "stale_s": 0.8,
    "search_yaw": 40,
}

_cfg = dict(DEFAULTS)
_cam = None
_model = None
_imgsz = 256
_running = False
_lock = threading.Lock()
_latest = None      # raw latest detection
_smoothed = None    # exponentially smoothed detection


def _ensure_route():
    subprocess.run(
        ["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
        capture_output=True
    )


def _load_model():
    ncnn = os.path.join(ROOT, "yolo11n_ncnn_model")
    meta = os.path.join(ncnn, "metadata.yaml")
    if os.path.isfile(meta):
        size = yaml.safe_load(open(meta)).get("imgsz", 640)
        imgsz = size[0] if isinstance(size, (list, tuple)) else size
        return YOLO(ncnn, task="detect"), imgsz
    return YOLO(os.path.join(ROOT, "yolo11n.pt")), 256


def _biggest_person(boxes, fw, fh):
    best, best_area = None, 0.0
    for b in boxes:
        x1, y1, x2, y2 = (float(v) for v in b.xyxy[0])
        area = (x2 - x1) * (y2 - y1)
        if area > best_area:
            best, best_area = (x1, y1, x2, y2), area
    if best is None:
        return None
    x1, y1, x2, y2 = best
    return {
        "cx": (x1 + x2) / 2 / fw,
        "top": y1 / fh,
        "bottom": y2 / fh,
        "h": (y2 - y1) / fh,
        "ts": time.time()
    }


def _smooth(prev, det):
    """Exponentially smooth detection values to reduce jumping."""
    if prev is None:
        return det
    a = SMOOTH
    return {
        "cx":     prev["cx"]     * (1 - a) + det["cx"]     * a,
        "top":    prev["top"]    * (1 - a) + det["top"]     * a,
        "bottom": prev["bottom"] * (1 - a) + det["bottom"]  * a,
        "h":      prev["h"]      * (1 - a) + det["h"]       * a,
        "ts":     det["ts"]
    }


def _detect_loop():
    global _latest, _smoothed
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
            if det:
                _smoothed = _smooth(_smoothed, det)
            else:
                _smoothed = None

        # --- live display ---
        display = frame.copy()
        fh_d, fw_d = display.shape[:2]
        s = _smoothed

        if s:
            # body box (using actual pixel coords from smoothed)
            x1 = int((s["cx"] - s["h"] * 0.3) * fw_d)
            x2 = int((s["cx"] + s["h"] * 0.3) * fw_d)
            y1 = int(s["top"] * fh_d)
            y2 = int(s["bottom"] * fh_d)
            cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # head dot — top 10% of bounding box
            head_x = int(s["cx"] * fw_d)
            head_y = int((s["top"] + s["h"] * 0.10) * fh_d)
            cv2.circle(display, (head_x, head_y), 8, (0, 0, 255), -1)
            cv2.circle(display, (head_x, head_y), 13, (255, 255, 255), 2)
            cv2.line(display, (head_x - 20, head_y), (head_x + 20, head_y), (0, 0, 255), 1)
            cv2.line(display, (head_x, head_y - 20), (head_x, head_y + 20), (0, 0, 255), 1)

            # target head height line (yellow)
            ty = int(_cfg["target_head_y"] * fh_d)
            cv2.line(display, (0, ty), (fw_d, ty), (0, 255, 255), 1)

            # distance indicator
            dist_pct = int(s["h"] * 100)
            target_pct = int(_cfg["target_dist_h"] * 100)
            dist_color = (0, 255, 0) if abs(s["h"] - _cfg["target_dist_h"]) < 0.1 else (0, 165, 255)
            cv2.putText(display, f"dist:{dist_pct}% target:{target_pct}%",
                        (8, fh_d - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, dist_color, 2)
            cv2.putText(display, "TRACKING", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cx, cy = fw_d // 2, fh_d // 2
            cv2.line(display, (cx - 30, cy), (cx + 30, cy), (0, 165, 255), 2)
            cv2.line(display, (cx, cy - 30), (cx, cy + 30), (0, 165, 255), 2)
            cv2.putText(display, "SEARCHING...", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        cv2.imshow("Drone Camera", display)
        cv2.waitKey(1)

        _dbg_count += 1
        if _dbg_count % 16 == 0:
            if s:
                print(f"[FOLLOW] person — cx:{s['cx']:.2f} h:{s['h']:.2f} top:{s['top']:.2f}")
            else:
                print("[FOLLOW] no person detected — searching...")


def start(state, config):
    global _cfg, _cam, _model, _imgsz, _running, _smoothed
    _cfg = {**DEFAULTS, **config.get("tuning", {}).get("follow", {})}
    _smoothed = None
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
    with _lock:
        det = _smoothed   # use smoothed detection, not raw

    if det is None or (time.time() - det["ts"]) > _cfg["stale_s"]:
        return CENTER, CENTER, CENTER, _stick(_cfg["search_yaw"], _cfg["max_yaw"])

    dz = _cfg["deadzone"]

    # head position (top 10% of box)
    head_x = det["cx"]
    head_y = det["top"] + det["h"] * 0.10

    # YAW: center the HEAD left/right (not body center)
    yaw_dev = _gated(head_x - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]

    # PITCH: hold follow distance based on body size
    body_cut = det["top"] < _cfg["edge_margin"] and det["bottom"] > (1 - _cfg["edge_margin"])
    if det["h"] > _cfg["too_close_h"] or body_cut:
        pitch_dev = -_cfg["max_pitch"]
    else:
        pitch_dev = _gated(_cfg["target_dist_h"] - det["h"], _cfg["pitch_gain"], dz) * _cfg["pitch_sign"]

    # THROTTLE: keep HEAD at target height (not body center)
    thr_dev = -_gated(head_y - _cfg["target_head_y"], _cfg["throttle_gain"], dz) * _cfg["throttle_sign"]

    roll = CENTER
    pitch = _stick(pitch_dev, _cfg["max_pitch"])
    throttle = _stick(thr_dev, _cfg["max_throttle"])
    yaw = _stick(yaw_dev, _cfg["max_yaw"])

    if not hasattr(controller, "_dbg"):
        controller._dbg = 0
    controller._dbg += 1
    if controller._dbg % 25 == 0:
        direction = "FORWARD" if pitch > 128 else "BACKWARD" if pitch < 128 else "HOLD"
        print(f"[FOLLOW] pitch={pitch} ({direction})  h={det['h']:.2f}  target={_cfg['target_dist_h']}")

    return roll, pitch, throttle, yaw