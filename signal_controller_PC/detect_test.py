r"""detect_test.py — see what the drone camera + pose model detect. NO flying.

Mirrors follow_human's vision: same camera, same pose model, same low-light boost, and the
same head box + distance reading — it just WATCHES instead of flying. Use it on the ground
to check FPS, low-light detection, the head/distance numbers, and whether anything is being
falsely detected (which is what makes the drone drive at "nobody").

It reads the SAME settings from config.json (tuning.follow), so what you see here is what
follow_human will act on. Run as Administrator (the camera needs the host route).

    python detect_test.py            # press 'q' to quit
"""
import json
import math
import os
import subprocess
import time

import cv2
import torch
from ultralytics import YOLO

from drone_camera import DroneCamera

RTSP = "rtsp://192.168.1.1:7070/webcam"
PERSON_CLASS = 0
NOSE, L_EYE, R_EYE, L_EAR, R_EAR = 0, 1, 2, 3, 4
L_SHO, R_SHO = 5, 6

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Pull the SAME tuning follow_human uses, so this shows exactly what it'll act on.
with open(os.path.join(HERE, "config.json")) as f:
    CFG = json.load(f).get("tuning", {}).get("follow", {})

MODEL = CFG.get("model", "yolo11m-pose.pt")
IMGSZ = CFG.get("imgsz", 640)
CONF = CFG.get("conf", 0.20)
KP_CONF = CFG.get("kp_conf", 0.25)
PERSON_KP_CONF = CFG.get("person_kp_conf", 0.5)
MIN_KP = CFG.get("min_kp", 6)
MIN_HEAD_FRAC = CFG.get("min_head_frac", 0.012)
LOW_LIGHT = CFG.get("low_light", True)
HEAD_WIDTH_FT = CFG.get("head_width_ft", 0.5)
HFOV = CFG.get("camera_hfov_deg", 30.0)
FRAME_W = CFG.get("frame_width_px", 640)
TARGET_FT = CFG.get("target_dist_ft", 11.0)

_clahe = cv2.createCLAHE(clipLimit=CFG.get("clahe_clip", 2.0), tileGridSize=(8, 8))


def ensure_route_and_reachable():
    subprocess.run(["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
                   capture_output=True)
    r = subprocess.run(["ping", "-n", "1", "-w", "2000", "192.168.1.1"], capture_output=True)
    return r.returncode == 0


def enhance(frame):
    if not LOW_LIGHT:
        return frame
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    return cv2.cvtColor(cv2.merge((_clahe.apply(l), a, b)), cv2.COLOR_LAB2BGR)


def head_metrics(kp):
    """Real-person head from keypoints -> (cx, cy, head_px) or None.

    Same filter as follow_human: reject unless there are enough STRONG keypoints (a statue
    won't have them), then head from the face, or the shoulders if turned away."""
    if int((kp[:, 2] >= PERSON_KP_CONF).sum()) < MIN_KP:
        return None                                          # not a real human skeleton
    face = {idx: (float(kp[idx][0]), float(kp[idx][1]))
            for idx in (NOSE, L_EYE, R_EYE, L_EAR, R_EAR) if kp[idx][2] >= KP_CONF}
    if len(face) >= 2:
        xs = [p[0] for p in face.values()]
        ys = [p[1] for p in face.values()]
        cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
        if L_EAR in face and R_EAR in face:
            hw = math.hypot(face[L_EAR][0] - face[R_EAR][0], face[L_EAR][1] - face[R_EAR][1])
        elif L_EYE in face and R_EYE in face:
            hw = math.hypot(face[L_EYE][0] - face[R_EYE][0], face[L_EYE][1] - face[R_EYE][1]) * 2.2
        else:
            hw = max(max(xs) - min(xs), 1.0)
    elif kp[L_SHO][2] >= KP_CONF and kp[R_SHO][2] >= KP_CONF:
        sx = (kp[L_SHO][0] + kp[R_SHO][0]) / 2.0
        sy = (kp[L_SHO][1] + kp[R_SHO][1]) / 2.0
        sw = math.hypot(kp[L_SHO][0] - kp[R_SHO][0], kp[L_SHO][1] - kp[R_SHO][1])
        cx, cy, hw = sx, sy - 0.6 * sw, 0.45 * sw
    else:
        return None
    hw = max(hw, 1.0)
    if hw < MIN_HEAD_FRAC * FRAME_W:
        return None
    return cx, cy, hw


def distance_ft(head_px):
    if not head_px or head_px <= 0:
        return None
    focal = (FRAME_W / 2) / math.tan(math.radians(HFOV) / 2)
    return (HEAD_WIDTH_FT * focal) / head_px


def main():
    global FRAME_W
    dev = "GPU (CUDA)" if torch.cuda.is_available() else "CPU"
    path = os.path.join(ROOT, MODEL)
    path = path if os.path.isfile(path) else MODEL
    print(f"[DET] model: {os.path.basename(path)} on {dev}  (conf={CONF}, low_light={LOW_LIGHT})")
    model = YOLO(path)

    print("[DET] route + reachability...")
    if not ensure_route_and_reachable():
        print("[DET] can't reach the drone. On FLOW-UFO? drone ON? phone off? running as Admin?")
        return

    print(f"[DET] opening {RTSP} ...")
    cam = DroneCamera(RTSP, debug=True)
    FRAME_W = cam.w                                  # auto-adjust distance math to this camera's width
    print(f"[DET] camera {cam.w}x{cam.h} — distance math auto-set to {cam.w}px wide")
    print("[DET] waiting for first frame...")
    t = time.time()
    while cam.read() is None:
        if time.time() - t > 15:
            print("[DET] no video after 15s — drone ON + streaming?")
            cam.stop()
            return
        time.sleep(0.1)

    print("[DET] LIVE — press 'q' to quit.")
    fps, t_prev = 0.0, time.time()
    try:
        while True:
            frame = cam.read()
            if frame is None:
                time.sleep(0.01)
                continue
            fh, fw = frame.shape[:2]
            proc = enhance(frame)
            res = model(proc, conf=CONF, classes=[PERSON_CLASS], imgsz=IMGSZ, verbose=False)[0]

            # dim box = every raw detection; RED head = the largest one that passes the
            # human-skeleton filter (so rejected statues stay dim with NO head box).
            boxes = res.boxes.xyxy.cpu().numpy() if (res.boxes is not None and res.boxes.xyxy is not None) else []
            kdata = res.keypoints.data.cpu().numpy() if (res.keypoints is not None and res.keypoints.data is not None) else None
            n = len(boxes)
            order = sorted(range(n), key=lambda i: (boxes[i][2] - boxes[i][0]) * (boxes[i][3] - boxes[i][1]),
                           reverse=True)
            real = 0
            chosen = None
            for i in order:
                x1, y1, x2, y2 = boxes[i]
                cv2.rectangle(proc, (int(x1), int(y1)), (int(x2), int(y2)), (0, 150, 0), 1)
                head = head_metrics(kdata[i]) if kdata is not None and i < len(kdata) else None
                if head is not None:
                    real += 1
                    if chosen is None:
                        chosen = head

            if chosen is not None:
                hx, hy, hw = chosen
                cv2.rectangle(proc, (int(hx - hw / 2), int(hy - hw / 2)),
                              (int(hx + hw / 2), int(hy + hw * 0.7)), (0, 0, 255), 2)
                cv2.circle(proc, (int(hx), int(hy)), 3, (0, 0, 255), -1)
                d = distance_ft(hw)
                if d is not None:
                    cv2.putText(proc, f"dist:{d:.1f}ft (target {TARGET_FT:.0f}ft)  head:{int(hw)}px",
                                (8, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            cv2.putText(proc, f"boxes:{n} real:{real}   {fps:4.1f} FPS   {dev}", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("Detect Test (q to quit)", proc)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        for _ in range(5):
            cv2.waitKey(1)
    print("[DET] done.")


if __name__ == "__main__":
    main()
