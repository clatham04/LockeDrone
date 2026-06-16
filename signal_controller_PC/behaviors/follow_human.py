r"""follow_human — the drone follows a person, tracking the HEAD with prediction.

Searches by spinning slowly until a person is found, then tracks them:
  - YAW:      keeps the head centered left/right
  - PITCH:    holds a follow distance, judged from the HEAD's apparent width
  - THROTTLE: keeps the head at a target height in the frame (target_head_y)

Why the head (not the body): up close the body box gets cut off, so any body-based
distance estimate breaks. We get the head from a POSE model's face keypoints
(nose/eyes/ears), independent of the body box.

STABILITY + PREDICTION (the important part):
The control loop runs at 25 Hz but detections arrive slower and are noisy. If we steered
straight off each raw detection, the drone would jitter (chasing noise) and fall behind a
moving person (steering toward a stale position). Instead we run an ALPHA-BETA filter: it
smooths the head position AND estimates its velocity. Between detections — and while you
move — the controller PREDICTS where your head is now (position + velocity x time), so it
stays locked on through motion and brief detection dropouts instead of losing you.

Distance: head real width (~0.5 ft, ear-to-ear) -> feet via the pinhole camera model.
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

# COCO pose keypoint indices for the head
NOSE, L_EYE, R_EYE, L_EAR, R_EAR = 0, 1, 2, 3, 4

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

DEFAULTS = {
    # --- model (pose, for head keypoints) ---
    "model": "yolo11m-pose.pt",
    "imgsz": 640,
    "conf": 0.25,
    "kp_conf": 0.30,

    # --- control ---
    "target_head_y": 0.35,
    "deadzone": 0.06,             # hold still inside this error (kills twitch when you're still)
    "yaw_gain": 90.0,
    "pitch_gain": 280.0,
    "throttle_gain": 130.0,
    "max_yaw": 45,
    "max_pitch": 50,
    "max_throttle": 30,
    "yaw_sign": 1,
    "pitch_sign": 1,
    "throttle_sign": 1,
    "search_yaw": 40,

    # --- tracking filter + prediction (alpha-beta) ---
    "alpha": 0.4,                 # how hard each detection corrects POSITION (0..1)
    "beta": 0.05,                 # how hard each detection corrects VELOCITY (lower = smoother)
    "hw_smooth": 0.3,             # head-width EMA (distance) smoothing
    "predict_cap_s": 0.25,        # never extrapolate position further ahead than this
    "max_vel": 0.6,               # clamp head velocity (frac/s) so prediction CAN'T run away
    "lock_hits": 6,               # need this many detections in a row before we trust prediction
    "lost_s": 1.2,                # no detection longer than this -> give up + search
    "reset_dt_s": 0.5,            # gap bigger than this -> re-init track (no velocity spike)

    # --- distance from HEAD width (pinhole camera model) ---
    "head_width_ft": 0.5,
    "camera_hfov_deg": 30.0,
    "frame_width_px": 640,
    "target_dist_ft": 11.0,
    "too_close_ft": 6.0,
    "head_width_frac": 0.55,      # body-box fallback only
}

_cfg = dict(DEFAULTS)
_cam = None
_model = None
_imgsz = 640
_running = False
_lock = threading.Lock()
_track = None    # filtered head track: {cx, cy, vx, vy, hw, box, src, ts} or None


def _ensure_route():
    """Pin the drone's IP via a host route on Windows."""
    subprocess.run(
        ["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
        capture_output=True
    )


def _load_model():
    """Load the pose model. Prefer a local file so it works OFFLINE on the drone's wifi."""
    name = _cfg.get("model", "yolo11m-pose.pt")
    local = os.path.join(ROOT, name)
    path = local if os.path.isfile(local) else name
    dev = "GPU (CUDA)" if torch.cuda.is_available() else "CPU"
    print(f"[FOLLOW] pose model: {os.path.basename(path)} on {dev}")
    if path == name and not os.path.isfile(local):
        print(f"[FOLLOW] (note: {name} not found in {ROOT} — needs internet to download.)")
    return YOLO(path), _cfg.get("imgsz", 640)


def _head_from_person(res, i, x1, y1, x2, y2, fw, fh):
    """Build raw head metrics for person i: prefer pose keypoints, fall back to the body box."""
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
        if L_EAR in pts and R_EAR in pts:
            hw = math.hypot(pts[L_EAR][0] - pts[R_EAR][0], pts[L_EAR][1] - pts[R_EAR][1])
        elif L_EYE in pts and R_EYE in pts:
            hw = math.hypot(pts[L_EYE][0] - pts[R_EYE][0], pts[L_EYE][1] - pts[R_EYE][1]) * 2.2
        else:
            hw = max(max(xs) - min(xs), 1.0)
        src = "head"
    else:
        cx, cy = (x1 + x2) / 2, y1 + (y2 - y1) * 0.10
        hw = (x2 - x1) * _cfg["head_width_frac"]
        src = "body"

    return {"cx": cx / fw, "cy": cy / fh, "head_w_px": max(hw, 1.0),
            "box": (x1 / fw, y1 / fh, x2 / fw, y2 / fh), "src": src, "ts": time.time()}


def _largest_person(res, fw, fh):
    """Largest detected person -> raw head metrics, or None."""
    boxes = res.boxes
    if boxes is None or boxes.xyxy is None or len(boxes) == 0:
        return None
    xyxy = boxes.xyxy.cpu().numpy()
    areas = (xyxy[:, 2] - xyxy[:, 0]) * (xyxy[:, 3] - xyxy[:, 1])
    i = int(areas.argmax())
    x1, y1, x2, y2 = (float(v) for v in xyxy[i])
    return _head_from_person(res, i, x1, y1, x2, y2, fw, fh)


def _update_track(track, det):
    """Alpha-beta filter: fold a new detection into a smoothed position + velocity estimate.

    'hits' counts consecutive detections; prediction is only trusted once it's high enough
    (a stable lock), so the velocity garbage from the search spin never steers us.
    """
    fresh = {"cx": det["cx"], "cy": det["cy"], "vx": 0.0, "vy": 0.0, "hits": 1,
             "hw": det["head_w_px"], "box": det["box"], "src": det["src"], "ts": det["ts"]}
    if track is None:
        return fresh
    dt = det["ts"] - track["ts"]
    if dt <= 0 or dt > _cfg["reset_dt_s"]:
        return fresh                                    # long gap -> restart, no velocity spike

    a, b = _cfg["alpha"], _cfg["beta"]
    # predict from the old state, then correct toward the measurement
    px, py = track["cx"] + track["vx"] * dt, track["cy"] + track["vy"] * dt
    rx, ry = det["cx"] - px, det["cy"] - py
    return {
        "cx": px + a * rx,
        "cy": py + a * ry,
        "vx": track["vx"] + (b / dt) * rx,
        "vy": track["vy"] + (b / dt) * ry,
        "hits": track["hits"] + 1,
        "hw": track["hw"] * (1 - _cfg["hw_smooth"]) + det["head_w_px"] * _cfg["hw_smooth"],
        "box": det["box"], "src": det["src"], "ts": det["ts"],
    }


def _predicted_pos(track):
    """Where the head is NOW. Until we have a stable LOCK (lock_hits), don't predict at all —
    just return the filtered position, so the search-spin's bogus velocity can't steer us.
    Once locked, extrapolate by the CLAMPED velocity (so it still can't run away). (cx, cy, age)."""
    age = max(time.time() - track["ts"], 0.0)
    if track["hits"] < _cfg["lock_hits"]:
        return track["cx"], track["cy"], age            # acquiring -> centre on the raw position
    pdt = min(age, _cfg["predict_cap_s"])
    vmax = _cfg["max_vel"]
    vx = max(-vmax, min(vmax, track["vx"]))
    vy = max(-vmax, min(vmax, track["vy"]))
    cx = min(1.0, max(0.0, track["cx"] + vx * pdt))
    cy = min(1.0, max(0.0, track["cy"] + vy * pdt))
    return cx, cy, age


def _distance_ft(head_px):
    """Distance in feet from the head's pixel width (pinhole camera model)."""
    if head_px <= 0:
        return None
    hfov_rad = math.radians(_cfg["camera_hfov_deg"])
    focal_px = (_cfg["frame_width_px"] / 2) / math.tan(hfov_rad / 2)
    return (_cfg["head_width_ft"] * focal_px) / head_px


def _draw_overlay(frame, track):
    """Draw the tracking overlay (head box at the predicted position, target line, distance)."""
    fh, fw = frame.shape[:2]

    if track is None:
        cx, cy = fw // 2, fh // 2
        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 165, 255), 2)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 165, 255), 2)
        cv2.putText(frame, "SEARCHING...", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        return frame

    pcx, pcy, age = _predicted_pos(track)
    locked = track["hits"] >= _cfg["lock_hits"]
    predicting = locked and age > 0.15

    bx1, by1, bx2, by2 = track["box"]
    cv2.rectangle(frame, (int(bx1 * fw), int(by1 * fh)), (int(bx2 * fw), int(by2 * fh)),
                  (0, 150, 0), 1)                       # body box (dim)

    hx, hy = int(pcx * fw), int(pcy * fh)               # predicted head position
    hw = int(track["hw"])
    color = (0, 200, 255) if predicting else (0, 0, 255)
    cv2.rectangle(frame, (hx - hw // 2, hy - hw // 2), (hx + hw // 2, hy + int(hw * 0.7)), color, 2)
    cv2.circle(frame, (hx, hy), 3, color, -1)

    ty = int(_cfg["target_head_y"] * fh)
    cv2.line(frame, (0, ty), (fw, ty), (0, 255, 255), 1)

    dist_ft = _distance_ft(track["hw"])
    if dist_ft is not None:
        c = (0, 255, 0) if abs(dist_ft - _cfg["target_dist_ft"]) < 1.0 else (0, 165, 255)
        cv2.putText(frame, f"dist:{dist_ft:.1f}ft tgt:{_cfg['target_dist_ft']:.1f}ft "
                    f"head:{int(track['hw'])}px [{track['src']}]",
                    (8, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    label = "ACQUIRING" if not locked else ("PREDICTING" if predicting else "TRACKING")
    lcolor = (0, 165, 255) if not locked else ((0, 200, 255) if predicting else (0, 255, 0))
    cv2.putText(frame, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, lcolor, 2)
    return frame


def _detect_loop():
    global _track
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
            if det is not None:
                _track = _update_track(_track, det)     # no detection -> keep predicting the old track
            tr = _track
        cv2.imshow("Drone Camera", _draw_overlay(frame, tr))
        cv2.waitKey(1)
        dbg += 1
        if dbg % 16 == 0:
            if tr is not None:
                print(f"[FOLLOW] head[{tr['src']}] cx:{tr['cx']:.2f} cy:{tr['cy']:.2f} "
                      f"v:({tr['vx']:+.2f},{tr['vy']:+.2f})/s w:{int(tr['hw'])}px")
            else:
                print("[FOLLOW] no person — searching...")


def start(state, config):
    global _cfg, _cam, _model, _imgsz, _running, _track
    _cfg = {**DEFAULTS, **config.get("tuning", {}).get("follow", {})}
    _track = None
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
    """Steer toward the PREDICTED head position -> (roll, pitch, throttle, yaw)."""
    with _lock:
        tr = _track

    # Lost for too long -> spin slowly in place to search
    if tr is None or (time.time() - tr["ts"]) > _cfg["lost_s"]:
        return CENTER, CENTER, CENTER, _stick(_cfg["search_yaw"], _cfg["max_yaw"])

    cx, cy, _age = _predicted_pos(tr)                   # where the head is NOW (predicted)
    dz = _cfg["deadzone"]

    # YAW: center the (predicted) head left/right
    yaw_dev = _gated(cx - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]

    # PITCH: hold follow distance (feet, from head width) — works even up close
    dist_ft = _distance_ft(tr["hw"])
    if dist_ft is None:
        pitch_dev = 0.0
    elif dist_ft < _cfg["too_close_ft"]:
        pitch_dev = -_cfg["max_pitch"]                  # too close -> full back-off
    else:
        ft_error = dist_ft - _cfg["target_dist_ft"]     # positive -> too far -> move forward
        pitch_dev = _gated(ft_error, _cfg["pitch_gain"] / _cfg["target_dist_ft"], 0.5) * _cfg["pitch_sign"]

    # THROTTLE: keep the (predicted) head at the target height
    thr_dev = -_gated(cy - _cfg["target_head_y"], _cfg["throttle_gain"], dz) * _cfg["throttle_sign"]

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
