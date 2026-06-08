r"""follow_human — the drone follows a person, vision-driven and flight-safe.

It watches the forward camera, finds the (largest) person, and turns their bounding box
into the four sticks. Goals, in the user's words:

  - NEVER run into the human. We hold a follow distance and hard-stop / back off if the
    box gets too big or the WHOLE BODY no longer fits the frame (that means too close).
  - Level with the head. We raise/lower so the head (top of the box) sits at a target
    height in the frame.
  - Follow where they go. YAW turns to keep them centred; PITCH moves in/out to hold
    distance. Walk away -> box shrinks -> move closer. Stand still -> no error -> HOVER.
  - Stay low + reachable. Altitude pushes are small and bounded.
  - Safe by default. No person, stale detection, or no frame yet -> HOVER. Never flies off.

Detection (YOLO, ~8 FPS) runs in its OWN thread; the 25 Hz control loop just reads the
latest box, so flight stays smooth and a video glitch simply -> hover.

Tuning lives in config.json under tuning.follow (so you can adjust outdoors without
editing code). If a stick drives the WRONG way during your tied-string test, flip the
matching *_sign (1 / -1) in config — that's the quick fix.
"""
import os
import threading
import time

import yaml
from ultralytics import YOLO

from drone_camera import DroneCamera

RTSP = "rtsp://192.168.1.1:7070/webcam"
PERSON_CLASS = 0
CENTER = 128

HERE = os.path.dirname(os.path.abspath(__file__))     # .../signal_controller/behaviors
ROOT = os.path.dirname(os.path.dirname(HERE))         # .../LockeDrone

# Defaults — overridden by config.json -> tuning.follow.
DEFAULTS = {
    "conf": 0.35,
    # targets (fractions of frame height/width)
    "target_dist_h": 0.60,    # hold the person's box at ~60% of frame height (follow distance)
    "too_close_h": 0.78,      # box taller than this = too close -> back off
    "target_head_y": 0.28,    # head (box top) should sit here -> "level with the head"
    "edge_margin": 0.04,      # box within this of top AND bottom = body cut off = too close
    "deadzone": 0.05,         # ignore errors smaller than this (no twitching when you're still)
    # proportional gains (stick units per unit fractional error)
    "yaw_gain": 110.0,
    "pitch_gain": 160.0,
    "throttle_gain": 130.0,
    # how far we ever push a stick from centre (gentle; throttle small so it stays low)
    "max_yaw": 45,
    "max_pitch": 32,
    "max_throttle": 24,
    # flip if a stick drives the wrong way during testing
    "yaw_sign": 1,
    "pitch_sign": 1,
    "throttle_sign": 1,
    "stale_s": 0.8,           # detection older than this -> hover
}

# module state: detector thread writes _latest, control loop reads it
_cfg = dict(DEFAULTS)
_cam = None
_model = None
_imgsz = 256
_running = False
_lock = threading.Lock()
_latest = None                # {"cx","top","bottom","h","ts"} in frame fractions, or None


def _load_model():
    """Prefer the fast ncnn export (read its baked imgsz); fall back to the .pt."""
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
    """Continuously detect the person and publish the latest box (its own thread)."""
    global _latest
    conf = _cfg["conf"]
    while _running:
        frame = _cam.read()
        if frame is None:
            time.sleep(0.02)
            continue
        fh, fw = frame.shape[:2]
        results = _model(frame, conf=conf, classes=[PERSON_CLASS], imgsz=_imgsz, verbose=False)
        with _lock:
            _latest = _biggest_person(results[0].boxes, fw, fh)


def start(state, config):
    global _cfg, _cam, _model, _imgsz, _running
    _cfg = {**DEFAULTS, **config.get("tuning", {}).get("follow", {})}
    print("[FOLLOW] starting camera + detector...")
    _cam = DroneCamera(RTSP)                       # adds the wlan0 route itself
    _model, _imgsz = _load_model()
    print(f"[FOLLOW] model ready @ {_imgsz}px — following. (Ctrl+C lands gently.)")
    _running = True
    threading.Thread(target=_detect_loop, daemon=True).start()


def stop(state, config):
    global _running
    _running = False
    if _cam:
        _cam.stop()
    print("[FOLLOW] stopped.")


def _stick(deviation, limit):
    """Clamp a deviation to +/- limit and return an absolute stick value (centred at 128)."""
    deviation = max(-limit, min(limit, deviation))
    return int(CENTER + deviation)


def _gated(error, gain, deadzone):
    """Proportional output, but 0 inside the deadzone (so we hold still when you do)."""
    return 0.0 if abs(error) < deadzone else gain * error


def controller(state):
    """Read the latest detection and return (roll, pitch, throttle, yaw). 128 = hold."""
    with _lock:
        det = _latest

    # Safety: nothing detected, or detection went stale -> HOVER. Never drift/fly off.
    if det is None or (time.time() - det["ts"]) > _cfg["stale_s"]:
        return CENTER, CENTER, CENTER, CENTER

    dz = _cfg["deadzone"]

    # --- YAW: turn to keep the person centred left/right ---
    yaw_dev = _gated(det["cx"] - 0.5, _cfg["yaw_gain"], dz) * _cfg["yaw_sign"]

    # --- PITCH: hold follow distance, and NEVER collide ---
    body_cut = det["top"] < _cfg["edge_margin"] and det["bottom"] > (1 - _cfg["edge_margin"])
    if det["h"] > _cfg["too_close_h"] or body_cut:
        pitch_dev = -_cfg["max_pitch"]             # too close / body cut off -> full safe back-off
    else:
        # small box (person far) -> positive error -> move forward; never past target distance
        pitch_dev = _gated(_cfg["target_dist_h"] - det["h"], _cfg["pitch_gain"], dz) * _cfg["pitch_sign"]

    # --- THROTTLE: stay level with the head (keep box top at target_head_y) ---
    thr_dev = -_gated(det["top"] - _cfg["target_head_y"], _cfg["throttle_gain"], dz) * _cfg["throttle_sign"]

    roll = CENTER                                  # no sideways strafing; yaw handles left/right
    pitch = _stick(pitch_dev, _cfg["max_pitch"])
    throttle = _stick(thr_dev, _cfg["max_throttle"])
    yaw = _stick(yaw_dev, _cfg["max_yaw"])
    return roll, pitch, throttle, yaw
