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

# COCO pose keypoint indices
NOSE, L_EYE, R_EYE, L_EAR, R_EAR = 0, 1, 2, 3, 4
L_SHO, R_SHO = 5, 6

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

DEFAULTS = {
    # --- model (pose, for head keypoints) ---
    "model": "yolo11m-pose.pt",
    "imgsz": 640,
    "conf": 0.20,                 # lower = detects in harder / low-light conditions
    "kp_conf": 0.25,              # min keypoint conf to USE a point for the head
    "person_kp_conf": 0.5,        # keypoints this confident count toward "is it a real human"
    "min_kp": 6,                  # need this many strong keypoints, or it's rejected (statue filter)
    "low_light": True,            # CLAHE contrast boost so it sees you in a dim room
    "clahe_clip": 2.0,

    # --- control ---
    "target_head_y": 0.35,
    "deadzone": 0.06,             # hold still inside this error (kills twitch when you're still)
    "yaw_gain": 90.0,
    "pitch_gain": 350.0,          # was 280 — more aggressive for outdoor use
    "throttle_gain": 130.0,
    "max_yaw": 45,
    "max_pitch": 65,              # was 50 — more aggressive for outdoor use
    "max_throttle": 30,
    "yaw_sign": 1,
    "pitch_sign": 1,
    "throttle_sign": 1,
    "search_yaw": 28,             # yaw amount during a search burst (gentle = less drift)
    "search_step_s": 0.45,        # how long each yaw burst lasts
    "search_hold_s": 0.8,         # pause between bursts to settle + look (no movement)

    # --- tracking filter + prediction (alpha-beta) ---
    "alpha": 0.4,                 # how hard each detection corrects POSITION (0..1)
    "beta": 0.05,                 # how hard each detection corrects VELOCITY (lower = smoother)
    "hw_smooth": 0.15,            # was 0.3 — more aggressive EMA to reject noisy head-width spikes
    "predict_cap_s": 0.25,        # never extrapolate position further ahead than this
    "max_vel": 0.6,               # clamp head velocity (frac/s) so prediction CAN'T run away
    "lock_hits": 6,               # need this many detections in a row before we trust prediction
    "fresh_s": 0.4,               # only move toward/away if a detection arrived within this (else hold)
    "lost_s": 1.8,                # was 1.2 — hold track longer before giving up + searching
    "reset_dt_s": 0.35,           # was 0.5 — tighter gap tolerance to avoid velocity spikes

    # --- distance from HEAD width (pinhole camera model) ---
    "head_width_ft": 0.5,
    "camera_hfov_deg": 30.0,
    "frame_width_px": 640,
    "target_dist_ft": 11.0,
    "too_close_ft": 5.0,
    "max_credible_dist_ft": 25.0, # readings above this are treated as noise (drive forward at full power)
    "implausible_pitch_frac": 1.0, # fraction of max_pitch to use when dist reading is implausible
    "head_width_frac": 0.55,      # body-box fallback only
    "min_head_px": 8,             # reject detections where head is smaller than this (too far / noise)
}

_cfg = dict(DEFAULTS)
_cam = None
_model = None
_imgsz = 640
_running = False
_lock = threading.Lock()
_track = None    # filtered head track: {cx, cy, vx, vy, hw, box, src, ts} or None
_clahe = None    # low-light contrast booster (created lazily)
_search_t = 0.0          # stepped-search phase timer
_search_yawing = False   # True = yaw burst, False = pause-and-look


def _enhance(frame):
    """Brighten/boost contrast for low-light detection (CLAHE on the luminance channel)."""
    global _clahe
    if not _cfg.get("low_light", True):
        return frame
    if _clahe is None:
        _clahe = cv2.createCLAHE(clipLimit=_cfg.get("clahe_clip", 2.0), tileGridSize=(8, 8))
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    return cv2.cvtColor(cv2.merge((_clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)


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


def _person_head(res, i, fw, fh):
    """Head metrics for detection i IF it's a real human; otherwise None.

    The owl-statue fix: a real person has many confident pose keypoints; a statue/sign
    mislabeled 'person' has only a few. So we require >= min_kp STRONG keypoints, then
    locate the head from the FACE (nose/eyes/ears) when visible, or the SHOULDERS when you
    face away. No more body-box fallback (that's what let it lock onto any box)."""
    if res.keypoints is None or res.keypoints.data is None or len(res.keypoints.data) <= i:
        return None
    kp = res.keypoints.data[i].cpu().numpy()                  # (17, 3): x, y, conf
    kc = _cfg["kp_conf"]
    if int((kp[:, 2] >= _cfg["person_kp_conf"]).sum()) < _cfg["min_kp"]:
        return None                                           # not a real human skeleton -> reject

    face = {idx: (float(kp[idx][0]), float(kp[idx][1]))
            for idx in (NOSE, L_EYE, R_EYE, L_EAR, R_EAR) if kp[idx][2] >= kc}
    if len(face) >= 2:                                        # facing us -> use the face
        xs = [p[0] for p in face.values()]
        ys = [p[1] for p in face.values()]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        if L_EAR in face and R_EAR in face:
            hw = math.hypot(face[L_EAR][0] - face[R_EAR][0], face[L_EAR][1] - face[R_EAR][1])
        elif L_EYE in face and R_EYE in face:
            hw = math.hypot(face[L_EYE][0] - face[R_EYE][0], face[L_EYE][1] - face[R_EYE][1]) * 2.2
        else:
            hw = max(max(xs) - min(xs), 1.0)
        src = "face"
    elif kp[L_SHO][2] >= kc and kp[R_SHO][2] >= kc:           # turned away -> head above shoulders
        sx = (kp[L_SHO][0] + kp[R_SHO][0]) / 2.0
        sy = (kp[L_SHO][1] + kp[R_SHO][1]) / 2.0
        sw = math.hypot(kp[L_SHO][0] - kp[R_SHO][0], kp[L_SHO][1] - kp[R_SHO][1])
        cx, cy, hw = sx, sy - 0.6 * sw, 0.45 * sw
        src = "body"
    else:
        return None                                           # can't locate a head -> reject

    hw = max(hw, 1.0)
    if hw < _cfg.get("min_head_px", 8):                       # too small -> too far / noise
        return None
    return {"cx": cx / fw, "cy": cy / fh, "head_w_px": hw, "src": src, "ts": time.time()}


def _largest_person(res, fw, fh):
    """Largest REAL person (passes the human-skeleton filter) -> head metrics, or None."""
    boxes = res.boxes
    if boxes is None or boxes.xyxy is None or len(boxes) == 0:
        return None
    xyxy = boxes.xyxy.cpu().numpy()
    order = sorted(range(len(xyxy)),
                   key=lambda i: (xyxy[i, 2] - xyxy[i, 0]) * (xyxy[i, 3] - xyxy[i, 1]),
                   reverse=True)
    for i in order:                                           # largest first; first valid one wins
        head = _person_head(res, i, fw, fh)
        if head is not None:
            x1, y1, x2, y2 = xyxy[i]
            head["box"] = (x1 / fw, y1 / fh, x2 / fw, y2 / fh)
            return head
    return None


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
        implausible = dist_ft > _cfg.get("max_credible_dist_ft", 25.0)
        c = (0, 0, 255) if implausible else (
            (0, 255, 0) if abs(dist_ft - _cfg["target_dist_ft"]) < 1.0 else (0, 165, 255))
        tag = " [!]" if implausible else ""
        cv2.putText(frame, f"dist:{dist_ft:.1f}ft tgt:{_cfg['target_dist_ft']:.1f}ft "
                    f"head:{int(track['hw'])}px [{track['src']}]{tag}",
                    (8, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    label = "ACQUIRING" if not locked else ("PREDICTING" if predicting else "TRACKING")
    lcolor = (0, 165, 255) if not locked else ((0, 200, 255) if predicting else (0, 255, 0))
    cv2.putText(frame, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, lcolor, 2)
    return frame


def _detect_loop():
    global _track
    conf = _cfg["conf"]
    dbg = 0
    fps, t_prev = 0.0, time.time()
    while _running:
        frame = _cam.read()
        if frame is None:
            time.sleep(0.02)
            continue
        fh, fw = frame.shape[:2]
        proc = _enhance(frame)                          # brighten for low-light detection
        res = _model(proc, conf=conf, classes=[PERSON_CLASS], imgsz=_imgsz, verbose=False)[0]
        det = _largest_person(res, fw, fh)
        with _lock:
            if det is not None:
                _track = _update_track(_track, det)     # no detection -> keep predicting the old track
            tr = _track

        now = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
        t_prev = now
        view = _draw_overlay(proc, tr)                  # show what the MODEL sees (enhanced)
        cv2.putText(view, f"{fps:4.1f} FPS", (fw - 110, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("Drone Camera", view)
        cv2.waitKey(1)

        dbg += 1
        if dbg % 16 == 0:
            state = f"locked hits:{tr['hits']}" if tr else "no person"
            print(f"[FOLLOW] {fps:.1f} FPS  |  {state}")


def start(state, config):
    global _cfg, _cam, _model, _imgsz, _running, _track, _clahe, _search_t, _search_yawing
    _cfg = {**DEFAULTS, **config.get("tuning", {}).get("follow", {})}
    _track = None
    _clahe = None
    _search_t, _search_yawing = 0.0, False
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


def _search_yaw_stick():
    """Stepped search: a short yaw burst, then a pause to settle + look — repeat. A pause
    between bursts lets the optical-flow hold re-lock (so it spins in PLACE instead of
    drifting in a circle) and gives a clean, non-blurred frame to detect a person."""
    global _search_t, _search_yawing
    now = time.time()
    phase = _cfg["search_step_s"] if _search_yawing else _cfg["search_hold_s"]
    if now - _search_t >= phase:
        _search_yawing = not _search_yawing
        _search_t = now
    return _stick(_cfg["search_yaw"], _cfg["max_yaw"]) if _search_yawing else CENTER


def controller(state):
    """Steer toward the PREDICTED head position -> (roll, pitch, throttle, yaw)."""
    with _lock:
        tr = _track

    # Lost for too long -> HOVER in place and step-spin to search (no translation at all)
    if tr is None or (time.time() - tr["ts"]) > _cfg["lost_s"]:
        return CENTER, CENTER, CENTER, _search_yaw_stick()

    cx, cy, age = _predicted_pos(tr)                    # where the head is NOW (predicted)
    locked = tr["hits"] >= _cfg["lock_hits"]
    dz = _cfg["deadzone"]

    # YAW: center the (predicted) head left/right
    yaw_dev = _gated(cx - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]

    # THROTTLE: keep the (predicted) head at the target height
    thr_dev = -_gated(cy - _cfg["target_head_y"], _cfg["throttle_gain"], dz) * _cfg["throttle_sign"]

    # PITCH (move toward / away): ONLY when locked AND freshly seen. Otherwise HOLD position
    # so it never drives forward while searching or coasting through a detection gap.
    dist_ft = _distance_ft(tr["hw"])
    max_credible = _cfg.get("max_credible_dist_ft", 25.0)
    implausible_frac = _cfg.get("implausible_pitch_frac", 0.6)

    if not (locked and age < _cfg["fresh_s"]):
        pitch_dev = 0.0                                 # not actively on someone -> stay put
    elif dist_ft is None:
        pitch_dev = 0.0
    elif dist_ft < _cfg["too_close_ft"]:
        pitch_dev = -_cfg["max_pitch"]                  # too close -> full back-off
    elif dist_ft > max_credible:
        # implausible reading (keypoint noise at range) — already far away and getting farther,
        # chase at full pitch to close ground
        pitch_dev = _cfg["max_pitch"]
    else:
        ft_error = dist_ft - _cfg["target_dist_ft"]     # positive -> too far -> move forward
        pitch_dev = _gated(ft_error, _cfg["pitch_gain"] / _cfg["target_dist_ft"], 0.5) * _cfg["pitch_sign"]

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
        implausible_tag = " [implausible->partial fwd]" if (dist_ft and dist_ft > max_credible) else ""
        print(f"[FOLLOW] pitch={pitch} ({direction})  dist={dist_str}  target={_cfg['target_dist_ft']}ft{implausible_tag}")

    return roll, pitch, throttle, yaw