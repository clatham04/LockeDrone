r"""follow_human — the drone follows a person, tracking the HEAD (pose keypoints).

Searches by spinning slowly until a person is found, then tracks them:
  - YAW:      keeps the head centered left/right
  - PITCH:    holds a follow distance, judged from the HEAD's apparent width
  - THROTTLE: keeps the head at a target height in the frame (target_head_y)

Why the head (not the body): up close the body box gets cut off by the frame, so any
body-based distance estimate breaks and the drone stops following. The head stays small
and fully in view much longer, so tracking head SIZE + head HEIGHT lets it keep following
even when you're close. We get the head from a POSE model's face keypoints
(nose/eyes/ears) — completely independent of the body box.

Distance: the head's real width (~ear-to-ear, ~0.5 ft) is roughly constant, so we convert
its pixel width to feet with the pinhole camera model. Detection is exponentially smoothed.
"""
import math
import os
import subprocess
import threading
import time

import cv2
import torch
import yaml
from ultralytics import YOLO

from drone_camera import DroneCamera

RTSP = "rtsp://192.168.1.1:7070/webcam"
PERSON_CLASS = 0
CENTER = 128
SMOOTH = 0.4   # exponential smoothing factor (0=no update, 1=no smoothing)

# COCO pose keypoint indices for the head
NOSE, L_EYE, R_EYE, L_EAR, R_EAR = 0, 1, 2, 3, 4

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

DEFAULTS = {
    # --- model (pose, for head keypoints) ---
    "model": "yolo11m-pose.pt",   # downloaded to the project root; PC GPU runs it easily
    "imgsz": 640,
    "conf": 0.25,                 # person confidence
    "kp_conf": 0.30,              # min keypoint confidence to trust a head point

    # --- control ---
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

    # --- distance from HEAD width (pinhole camera model) ---
    # Adult head is ~0.5 ft (ear to ear) and stays roughly constant from any angle.
    # focal_px = (frame_width_px/2)/tan(hfov/2);  distance_ft = head_width_ft*focal_px/head_px
    "head_width_ft": 0.5,
    "camera_hfov_deg": 30.0,
    "frame_width_px": 640,
    "target_dist_ft": 11.0,
    "too_close_ft": 6.0,
    "head_width_frac": 0.55,      # ONLY for the body-box fallback when the head isn't visible
}

_cfg = dict(DEFAULTS)
_cam = None
_model = None
_imgsz = 640
_running = False
_lock = threading.Lock()
_smoothed = None    # exponentially smoothed detection (or None)


def _ensure_route():
    """Pin the drone's IP via a host route on Windows."""
    subprocess.run(
        ["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
        capture_output=True
    )


def _load_model():
    """Load the pose model. Prefer a local file in the project root so it works OFFLINE
    on the drone's wifi (ultralytics would otherwise try to download it)."""
    name = _cfg.get("model", "yolo11m-pose.pt")
    local = os.path.join(ROOT, name)
    path = local if os.path.isfile(local) else name
    dev = "GPU (CUDA)" if torch.cuda.is_available() else "CPU"
    print(f"[FOLLOW] pose model: {os.path.basename(path)} on {dev}")
    if path == name and not os.path.isfile(local):
        print(f"[FOLLOW] (note: {name} not found in {ROOT} — needs internet to download.)")
    return YOLO(path), _cfg.get("imgsz", 640)


def _head_from_person(res, i, x1, y1, x2, y2, fw, fh):
    """Build head metrics for person i: prefer pose keypoints, fall back to the body box."""
    kp = None
    if res.keypoints is not None and res.keypoints.data is not None and len(res.keypoints.data) > i:
        kp = res.keypoints.data[i].cpu().numpy()        # (17, 3): x, y, conf

    pts = {}
    if kp is not None:
        for idx in (NOSE, L_EYE, R_EYE, L_EAR, R_EAR):
            x, y, c = kp[idx]
            if c >= _cfg["kp_conf"]:
                pts[idx] = (float(x), float(y))

    if len(pts) >= 2:
        xs = [p[0] for p in pts.values()]
        ys = [p[1] for p in pts.values()]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        if L_EAR in pts and R_EAR in pts:               # best: real head width
            hw = math.hypot(pts[L_EAR][0] - pts[R_EAR][0], pts[L_EAR][1] - pts[R_EAR][1])
        elif L_EYE in pts and R_EYE in pts:             # eyes span ~0.45 of head width
            hw = math.hypot(pts[L_EYE][0] - pts[R_EYE][0], pts[L_EYE][1] - pts[R_EYE][1]) * 2.2
        else:
            hw = max(max(xs) - min(xs), 1.0)
        src = "head"
    else:                                               # no trustworthy head points
        cx, cy = (x1 + x2) / 2, y1 + (y2 - y1) * 0.10
        hw = (x2 - x1) * _cfg["head_width_frac"]
        src = "body"

    return {"cx": cx / fw, "cy": cy / fh, "head_w_px": max(hw, 1.0),
            "box": (x1 / fw, y1 / fh, x2 / fw, y2 / fh), "src": src, "ts": time.time()}


def _largest_person(res, fw, fh):
    """Largest detected person -> head metrics, or None."""
    boxes = res.boxes
    if boxes is None or boxes.xyxy is None or len(boxes) == 0:
        return None
    xyxy = boxes.xyxy.cpu().numpy()
    areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
    i = int(areas.argmax())
    x1, y1, x2, y2 = (float(v) for v in xyxy[i])
    return _head_from_person(res, i, x1, y1, x2, y2, fw, fh)


def _smooth(prev, det):
    """Exponentially smooth the head values to reduce jitter."""
    if prev is None:
        return det
    a = SMOOTH
    return {
        "cx": prev["cx"] * (1 - a) + det["cx"] * a,
        "cy": prev["cy"] * (1 - a) + det["cy"] * a,
        "head_w_px": prev["head_w_px"] * (1 - a) + det["head_w_px"] * a,
        "box": det["box"],
        "src": det["src"],
        "ts": det["ts"],
    }


def _estimate_distance_ft(det):
    """Distance in feet from the head's pixel width (pinhole camera model)."""
    head_px = det.get("head_w_px", 0)
    if head_px <= 0:
        return None
    hfov_rad = math.radians(_cfg["camera_hfov_deg"])
    focal_px = (_cfg["frame_width_px"] / 2) / math.tan(hfov_rad / 2)
    return (_cfg["head_width_ft"] * focal_px) / head_px


def _draw_overlay(frame, det):
    """Draw the tracking overlay (body box, head box, target line, distance) onto frame."""
    fh, fw = frame.shape[:2]

    if det is None:
        cx, cy = fw // 2, fh // 2
        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 165, 255), 2)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 165, 255), 2)
        cv2.putText(frame, "SEARCHING...", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        return frame

    bx1, by1, bx2, by2 = det["box"]
    cv2.rectangle(frame, (int(bx1 * fw), int(by1 * fh)), (int(bx2 * fw), int(by2 * fh)),
                  (0, 150, 0), 1)                       # body box (dim — not what we track)

    # head box (what we actually track), sized by head_w_px, centered on the head
    hx, hy = int(det["cx"] * fw), int(det["cy"] * fh)
    hw = int(det["head_w_px"])
    cv2.rectangle(frame, (hx - hw // 2, hy - hw // 2), (hx + hw // 2, hy + int(hw * 0.7)),
                  (0, 0, 255), 2)
    cv2.circle(frame, (hx, hy), 3, (0, 0, 255), -1)

    ty = int(_cfg["target_head_y"] * fh)                # target head-height line
    cv2.line(frame, (0, ty), (fw, ty), (0, 255, 255), 1)

    dist_ft = _estimate_distance_ft(det)
    target_ft = _cfg["target_dist_ft"]
    if dist_ft is not None:
        color = (0, 255, 0) if abs(dist_ft - target_ft) < 1.0 else (0, 165, 255)
        cv2.putText(frame, f"dist:{dist_ft:.1f}ft target:{target_ft:.1f}ft "
                    f"head:{int(det['head_w_px'])}px [{det['src']}]",
                    (8, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.putText(frame, "TRACKING", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return frame


def _detect_loop():
    global _smoothed
    conf = _cfg["conf"]
    dbg = 0
    while _running:
        frame = _cam.read()
        if frame is None:
            time.sleep(0.02)
            continue
        fh, fw = frame.shape[:2]
        res = _model(frame, conf=conf, classes=[PERSON_CLASS], imgsz=_imgsz, verbose=False)[0]
        det = _largest_person(res, fw, fh)
        with _lock:
            _smoothed = _smooth(_smoothed, det) if det else None
            s = _smoothed
        cv2.imshow("Drone Camera", _draw_overlay(frame, s))
        cv2.waitKey(1)
        dbg += 1
        if dbg % 16 == 0:
            if s:
                print(f"[FOLLOW] head[{s['src']}] cx:{s['cx']:.2f} cy:{s['cy']:.2f} "
                      f"w:{int(s['head_w_px'])}px")
            else:
                print("[FOLLOW] no person — searching...")


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
    for _ in range(5):
        cv2.waitKey(1)  # flush window-close events on Windows
    print("[FOLLOW] stopped.")


def _stick(deviation, limit):
    """Clamp a deviation to +/- limit and return an absolute stick value (centered at 128)."""
    deviation = max(-limit, min(limit, deviation))
    return int(CENTER + deviation)


def _gated(error, gain, deadzone):
    """Proportional output, but 0 inside the deadzone (so we hold still when you do)."""
    return 0.0 if abs(error) < deadzone else gain * error


def controller(state):
    """Read the latest smoothed head detection and return (roll, pitch, throttle, yaw)."""
    with _lock:
        det = _smoothed

    # Lost: spin slowly in place to search
    if det is None or (time.time() - det["ts"]) > _cfg["stale_s"]:
        return CENTER, CENTER, CENTER, _stick(_cfg["search_yaw"], _cfg["max_yaw"])

    dz = _cfg["deadzone"]

    # YAW: center the head left/right
    yaw_dev = _gated(det["cx"] - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]

    # PITCH: hold follow distance (feet, from head width) — works even when close
    dist_ft = _estimate_distance_ft(det)
    if dist_ft is None:
        pitch_dev = 0.0
    elif dist_ft < _cfg["too_close_ft"]:
        pitch_dev = -_cfg["max_pitch"]                  # too close -> full back-off
    else:
        ft_error = dist_ft - _cfg["target_dist_ft"]     # positive -> too far -> move forward
        pitch_dev = _gated(ft_error, _cfg["pitch_gain"] / _cfg["target_dist_ft"], 0.5) * _cfg["pitch_sign"]

    # THROTTLE: keep the head at the target height
    thr_dev = -_gated(det["cy"] - _cfg["target_head_y"], _cfg["throttle_gain"], dz) * _cfg["throttle_sign"]

    roll = CENTER
    pitch = _stick(pitch_dev, _cfg["max_pitch"])
    throttle = _stick(thr_dev, _cfg["max_throttle"])
    yaw = _stick(yaw_dev, _cfg["max_yaw"])

    if not hasattr(controller, "_dbg"):
        controller._dbg = 0
    controller._dbg += 1
    if controller._dbg % 25 == 0:
        direction = "FORWARD" if pitch > 128 else "BACKWARD" if pitch < 128 else "HOLD"
        dist_str = f"{dist_ft:.1f}ft" if dist_ft is not None else "?"
        print(f"[FOLLOW] pitch={pitch} ({direction})  dist={dist_str}  target={_cfg['target_dist_ft']}ft")

    return roll, pitch, throttle, yaw
