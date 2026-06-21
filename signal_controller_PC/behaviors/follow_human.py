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
    "person_kp_conf": 0.4,        # keypoints this confident count toward "is it a real human"
    "min_kp": 5,                  # need this many strong keypoints, or it's rejected (statue filter)
    "low_light": True,            # CLAHE contrast boost so it sees you in a dim room
    "clahe_clip": 2.0,

    # --- control ---
    "target_head_y": 0.5,         # follow target: head near center = drone level with your head
    "deadzone": 0.06,             # hold still inside this error (kills twitch when you're still)
    "yaw_gain": 55.0,             # GENTLE turn — hard turns swing the zoomed camera and blur detection
    "pitch_gain": 350.0,          # was 280 — more aggressive for outdoor use
    "throttle_gain": 90.0,        # gentle altitude tracking — avoid the up/down bounce
    "vert_deadzone": 0.12,        # WIDE vertical hold band: don't chase small head-y changes
    "edge_at": 0.30,              # head past this far from centre = nearing the frame edge
    "edge_boost": 1.7,            # gain multiplier at the edge — yank it back before it's lost
    "max_yaw": 60,                # turn faster so you don't drift out before it centers
    "max_pitch": 65,              # was 50 — more aggressive for outdoor use
    "max_throttle": 30,           # gentle follow-altitude steps (the climb uses climb_throttle)
    "yaw_sign": 1,
    "pitch_sign": 1,
    "throttle_sign": 1,

    # --- wind stabilization ---
    "roll_gain": 70.0,            # strafe sideways to hold position against side-wind
    "roll_sign": 1,               # flip to -1 if it strafes the WRONG way
    "max_roll": 45,
    "wind_assist": True,          # integral "trim" that builds up to cancel a STEADY wind
    "i_gain": 0.03,               # how fast the wind integral builds (bigger = faster, riskier)
    "i_clamp": 20,                # max push the integral can add per axis (anti-windup; keeps it safe)
    "search_yaw": 25,             # SLOW yaw step (smaller = less blur, catches people reliably)
    "search_step_s": 0.4,         # short burst so it doesn't whip past people
    "search_hold_s": 0.9,         # long PAUSE to hold still and DETECT/LOCK (this is when it sees you)
    "climb_seconds": 6.0,         # fly HIGH first: climb for this long while searching for a person
    "climb_throttle": 160,        # gentle climb (128 = hold) so the rise doesn't blur detection

    # --- tracking filter + prediction (alpha-beta) ---
    "alpha": 0.4,                 # how hard each detection corrects POSITION (0..1)
    "beta": 0.15,                 # VELOCITY correction. Higher = velocity (and the lead) COLLAPSES
                                  #   fast when you STOP, so it stops yawing the instant you do.
    "hw_smooth": 0.07,            # heavy EMA on head width -> stable distance (kills back-and-forth)
    "dist_deadzone_ft": 2.0,      # hold position unless you're clearly off the target distance
    "lead_s": 0.4,                # how far AHEAD to predict your motion — bigger = turns to future sooner
    "vel_deadband": 0.12,         # ignore velocity below this when leading -> NO bounce when still
    "vhw_deadband_frac": 0.10,    # ignore head-width velocity below this (frac of head size) when still
    "predict_cap_s": 0.6,         # cap on how far ahead we ever extrapolate (anti-runaway)
    "max_vel": 0.6,               # clamp head velocity (frac/s) so prediction CAN'T run away
    "lock_hits": 6,               # need this many detections in a row before we trust prediction
    "prefer_hits": 3,             # need this many hits before sticky-tracking trusts the current track
                                   # (prevents getting anchored to a bad point right after a reset)
    "fresh_s": 0.4,               # only move toward/away if a detection arrived within this (else hold)
    "lost_s": 1.8,                # was 1.2 — hold track longer before giving up + searching
    "reset_dt_s": 0.5,            # gap bigger than this -> re-init track (no velocity spike)

    # --- distance from HEAD width (pinhole camera model) ---
    # Resolution (frame_width_px) AUTO-adjusts to the camera at startup; head_width_ft is
    # physical. camera_hfov_deg is the ONE per-LENS value that can't be read from the video
    # — CALIBRATE it: stand a measured distance away, run detect_test, and tune this until the
    # on-screen 'dist' matches reality. Then any camera follows at the right distance.
    "head_width_ft": 0.5,         # avg adult head width (ft) — physical, not camera-dependent
    "camera_hfov_deg": 30.0,      # << per-lens calibration (see above). Wider lens = bigger number.
    "frame_width_px": 640,        # AUTO-set from the live camera at startup (this is only a fallback)
    "target_dist_ft": 24.0,       # follow FARTHER so you fill less of the (zoomed) frame
    "too_close_ft": 8.0,
    "max_credible_dist_ft": 25.0, # readings above this are treated as noise (drive forward at full power)
    "implausible_pitch_frac": 0.5, # fraction of max_pitch when dist reading is implausible (no full lurch)
    "head_width_frac": 0.55,      # body-box fallback only
    "min_head_frac": 0.012,       # reject heads smaller than this FRACTION of the frame (resolution-safe)
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
_i = {"roll": 0.0, "pitch": 0.0, "thr": 0.0}   # wind-trim integrals per axis
_flight_start = None     # set on the first control tick (right after takeoff)


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
    if hw < _cfg.get("min_head_frac", 0.012) * fw:           # too small (frac of frame) -> far/noise
        return None
    return {"cx": cx / fw, "cy": cy / fh, "head_w_px": hw, "src": src, "ts": time.time()}


def _largest_person(res, fw, fh, prefer=None):
    """Best REAL person -> head metrics, or None. With a current track (prefer=(cx,cy)) it
    STICKS to the person nearest that point instead of hopping to another box each frame
    (stops it flipping between you and a statue/second person)."""
    boxes = res.boxes
    if boxes is None or boxes.xyxy is None or len(boxes) == 0:
        return None
    xyxy = boxes.xyxy.cpu().numpy()
    order = sorted(range(len(xyxy)),
                   key=lambda i: (xyxy[i, 2] - xyxy[i, 0]) * (xyxy[i, 3] - xyxy[i, 1]),
                   reverse=True)
    cands = []
    for i in order:                                           # largest first
        head = _person_head(res, i, fw, fh)
        if head is not None:
            x1, y1, x2, y2 = xyxy[i]
            head["box"] = (x1 / fw, y1 / fh, x2 / fw, y2 / fh)
            cands.append(head)
    if not cands:
        return None
    if prefer is not None:                                    # already locked -> stay on this target
        return min(cands, key=lambda h: (h["cx"] - prefer[0]) ** 2 + (h["cy"] - prefer[1]) ** 2)
    return cands[0]                                           # no track yet -> largest valid person


def _update_track(track, det):
    """Alpha-beta filter: fold a new detection into a smoothed position + velocity estimate.

    'hits' counts consecutive detections; prediction is only trusted once it's high enough
    (a stable lock), so the velocity garbage from the search spin never steers us.
    """
    fresh = {"cx": det["cx"], "cy": det["cy"], "vx": 0.0, "vy": 0.0, "hits": 1,
             "hw": det["head_w_px"], "vhw": 0.0, "box": det["box"], "src": det["src"], "ts": det["ts"]}
    if track is None:
        return fresh
    dt = det["ts"] - track["ts"]
    if dt <= 0 or dt > _cfg["reset_dt_s"]:
        return fresh                                    # long gap -> restart, no velocity spike

    a, b = _cfg["alpha"], _cfg["beta"]
    # predict from the old state, then correct toward the measurement
    px, py = track["cx"] + track["vx"] * dt, track["cy"] + track["vy"] * dt
    rx, ry = det["cx"] - px, det["cy"] - py
    new_hw = track["hw"] * (1 - _cfg["hw_smooth"]) + det["head_w_px"] * _cfg["hw_smooth"]
    vhw = track.get("vhw", 0.0) * 0.7 + ((new_hw - track["hw"]) / dt) * 0.3   # head-width velocity (px/s)
    return {
        "cx": px + a * rx,
        "cy": py + a * ry,
        "vx": track["vx"] + (b / dt) * rx,
        "vy": track["vy"] + (b / dt) * ry,
        "hits": track["hits"] + 1,
        "hw": new_hw,
        "vhw": vhw,
        "box": det["box"], "src": det["src"], "ts": det["ts"],
    }


def _predicted_pos(track):
    """Where the head is NOW. Until we have a stable LOCK (lock_hits), don't predict at all —
    just return the filtered position, so the search-spin's bogus velocity can't steer us.
    Once locked, extrapolate by the CLAMPED velocity (so it still can't run away). (cx, cy, age)."""
    age = max(time.time() - track["ts"], 0.0)
    if track["hits"] < _cfg["lock_hits"]:
        return track["cx"], track["cy"], age            # acquiring -> centre on the raw position
    # LEAD ahead of your motion (not just the tiny gap since the last detection), so it
    # anticipates your walk instead of always chasing where you just were.
    pdt = min(age + _cfg.get("lead_s", 0.3), _cfg["predict_cap_s"])
    vmax = _cfg["max_vel"]
    vd = _cfg.get("vel_deadband", 0.12)
    # LEAD only HORIZONTALLY, and only when you're actually MOVING. Tiny velocity is just
    # detection jitter while you stand still. We do NOT predict vertically: people don't fly,
    # and the drone's own up/down bounce makes the head LOOK like it jumps in frame -> a bogus
    # "head flying up" lead. So vertical stays pinned to the ACTUAL head position. (The camera
    # is tilted ~15 deg down, so vertical framing is geometry, not motion — never extrapolate it.)
    vx = track["vx"] if abs(track["vx"]) >= vd else 0.0
    vx = max(-vmax, min(vmax, vx))
    cx = min(1.0, max(0.0, track["cx"] + vx * pdt))
    return cx, track["cy"], age


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
        with _lock:
            # Only "stick" to the current track once it's a CONFIRMED lock (hits >= prefer_hits).
            # Right after a reset (hits 1-2) the track might be noise, so don't let it anchor
            # the search to a bad point — fall back to picking the largest valid person instead,
            # which is much more likely to be you if you're nearby.
            prefer = (_track["cx"], _track["cy"]) if (
                _track is not None and _track["hits"] >= _cfg.get("prefer_hits", 3)
            ) else None
        det = _largest_person(res, fw, fh, prefer)      # stick to the current target if we have one
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
    global _cfg, _cam, _model, _imgsz, _running, _track, _clahe, _search_t, _search_yawing, _flight_start
    _cfg = {**DEFAULTS, **config.get("tuning", {}).get("follow", {})}
    _track = None
    _clahe = None
    _search_t, _search_yawing = 0.0, False
    _flight_start = None
    _i["roll"] = _i["pitch"] = _i["thr"] = 0.0
    _ensure_route()
    print("[FOLLOW] starting camera + detector...")
    _cam = DroneCamera(RTSP)
    _cfg["frame_width_px"] = _cam.w           # AUTO-adjust the distance math to this camera's resolution
    print(f"[FOLLOW] camera {_cam.w}x{_cam.h} — distance math auto-set to {_cam.w}px wide")
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


def _center(error, gain, deadzone):
    """Center an axis with a BUFFER ZONE + soft wall. Hold inside the deadzone (small moves
    are free, no chasing), proportional outside it, and RAMP the gain up as the head nears
    the frame EDGE so it's pulled back hard before it slips off-screen and we lose lock."""
    a = abs(error)
    if a < deadzone:
        return 0.0
    edge_at = _cfg.get("edge_at", 0.30)               # |error| beyond this = nearing the edge
    if a > edge_at:
        frac = min((a - edge_at) / (0.5 - edge_at), 1.0)     # 0 at edge_at -> 1 at the frame edge
        gain *= 1.0 + frac * (_cfg.get("edge_boost", 2.0) - 1.0)
    return gain * error


def _pi(name, p_dev, active):
    """Proportional output + a leaky, clamped INTEGRAL of it (wind trim). A steady wind
    leaves a persistent correction (p_dev), which the integral accumulates into a sustained
    push that cancels the wind; it leaks away once you're back on target. We integrate the
    P OUTPUT (not the raw error), so the integral is always the SAME direction as P — it can
    never flip a sign. i_clamp bounds it so a gust can't wind it up dangerously."""
    if not _cfg.get("wind_assist", True):
        return p_dev
    if active:
        acc = _i[name] * 0.99 + p_dev * _cfg["i_gain"]
        _i[name] = max(-_cfg["i_clamp"], min(_cfg["i_clamp"], acc))
    else:
        _i[name] *= 0.9                                 # bleed off when not actively tracking
    return p_dev + _i[name]


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
    global _flight_start
    if _flight_start is None:
        _flight_start = time.time()
    # CLIMB to flight height during the first climb_seconds of flight — while STILL
    # searching/following, so it can spot + lock onto someone on the way up.
    climbing = (time.time() - _flight_start) < _cfg.get("climb_seconds", 0)
    climb_thr = _cfg.get("climb_throttle", 175)

    with _lock:
        tr = _track

    # Lost for too long -> climb (if still climbing) and step-spin to search.
    if tr is None or (time.time() - tr["ts"]) > _cfg["lost_s"]:
        _i["roll"] = _i["pitch"] = _i["thr"] = 0.0      # drop wind trims when we lose the person
        return CENTER, CENTER, (climb_thr if climbing else CENTER), _search_yaw_stick()

    pcx, _pcy, age = _predicted_pos(tr)                 # PREDICTED (lead) head position
    cx, cy = tr["cx"], tr["cy"]                          # ACTUAL head position
    locked = tr["hits"] >= _cfg["lock_hits"]
    active = locked and age < _cfg["fresh_s"]           # actively on someone -> let wind trim build
    dz = _cfg["deadzone"]

    # YAW + ROLL aim at the PREDICTED (future) horizontal position -> the drone turns toward
    # where you're HEADING, so your face stays centered even when you move fast (instead of
    # lagging the actual position and losing you). The velocity dead-band zeroes the lead when
    # you're still, so it won't bounce in place; the edge-boost still yanks it back near a edge.
    yaw_dev = _center(pcx - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]
    roll_p = _center(pcx - 0.5, _cfg["roll_gain"], dz)
    roll_dev = _pi("roll", roll_p, active) * _cfg["roll_sign"]

    # THROTTLE: we're following someone now -> track HEAD HEIGHT (descend from the high search
    # altitude to be level with their head). P-only, no integral, so it can't wind up and
    # climb away. The high search altitude is only used while LOOKING (the search branch above).
    _i["thr"] *= 0.9
    vdz = _cfg.get("vert_deadzone", 0.12)               # WIDE vertical dead-band: hold altitude
    thr_dev = -_center(cy - _cfg["target_head_y"], _cfg["throttle_gain"], vdz) * _cfg["throttle_sign"]
    throttle = _stick(thr_dev, _cfg["max_throttle"])    # unless your head is well off — no bounce

    # PITCH (toward / away): only when actively locked; otherwise HOLD (bleed the trim).
    # LEAD the distance: project the head size forward by your walking speed so it starts
    # moving to meet you instead of waiting for you to be clearly far, then reacting late.
    _vhw = tr.get("vhw", 0.0)
    if abs(_vhw) < _cfg.get("vhw_deadband_frac", 0.10) * tr["hw"]:
        _vhw = 0.0                                       # ignore head-width noise when you're still
    _lead = max(-0.4 * tr["hw"], min(0.4 * tr["hw"], _vhw * _cfg.get("lead_s", 0.3)))
    dist_ft = _distance_ft(max(tr["hw"] + _lead, 1.0))
    max_credible = _cfg.get("max_credible_dist_ft", 25.0)
    implausible_frac = _cfg.get("implausible_pitch_frac", 0.6)

    if not active:
        _i["pitch"] *= 0.9
        pitch_dev = 0.0                                 # not actively on someone -> stay put
    elif dist_ft is None:
        _i["pitch"] *= 0.9
        pitch_dev = 0.0
    elif dist_ft < _cfg["too_close_ft"]:
        _i["pitch"] *= 0.9
        pitch_dev = -_cfg["max_pitch"]                  # too close -> full back-off
    elif dist_ft > max_credible:
        _i["pitch"] *= 0.9
        # implausible reading (keypoint noise at range): nudge forward at the configured
        # fraction, NOT full pitch — lurching at max_pitch on a noisy frame jerks it around.
        pitch_dev = _cfg["max_pitch"] * implausible_frac
    else:
        ft_error = dist_ft - _cfg["target_dist_ft"]     # positive -> too far -> move forward
        # wide dead-band so head-width jitter doesn't creep it back and forth; + wind trim.
        pitch_p = _gated(ft_error, _cfg["pitch_gain"] / _cfg["target_dist_ft"],
                         _cfg.get("dist_deadzone_ft", 2.0))
        pitch_dev = _pi("pitch", pitch_p, True) * _cfg["pitch_sign"]

    roll = _stick(roll_dev, _cfg["max_roll"])
    pitch = _stick(pitch_dev, _cfg["max_pitch"])
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