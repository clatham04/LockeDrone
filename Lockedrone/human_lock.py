"""Locke Drone — human tracking main loop.

Read a frame -> detect the nearest person -> compute telemetry -> draw + report.
Everything you'd want to tune lives in config.py. Read this file top-to-bottom and
each step is a named function call; nothing hidden.
"""
import time

import cv2

import config
from camera import Camera
from detector import load_model, detect_person
from telemetry import distance_to_human, distance_to_ground


def fmt(value):
    """Format a number for the dashboard, or '--' when it's missing."""
    return f"{value:.1f}" if value is not None else "--"


def compute_telemetry(box, frame_w, frame_h):
    """Turn a detection box into physical telemetry (distance, altitude, offsets)."""
    x1, y1, x2, y2, conf = box
    box_w, box_h = x2 - x1, y2 - y1

    distance = distance_to_human(
        box_w, box_h,
        config.KNOWN_WIDTH_CM, config.KNOWN_HEIGHT_CM, config.FOCAL_LENGTH_PX,
    )
    altitude = distance_to_ground(distance, y2, frame_h, config.CAMERA_TILT_DEG)

    center_x = x1 + box_w / 2
    head = (int(center_x), int(y1 + box_h * 0.10))   # lock point: 10% down from the top
    offset_x = center_x - frame_w / 2                # how far off-centre the target is

    return {
        "confidence": conf,
        "distance": distance,
        "altitude": altitude,
        "offset_x": offset_x,
        "head": head,
    }


def draw_overlays(frame, box, head):
    """Draw the bounding box, the red head-lock dot, and a label."""
    x1, y1, x2, y2, _ = box
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, head, 7, (0, 0, 255), -1)
    cv2.putText(frame, "LOCK ON", (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


def render_dashboard(status, fps, telem):
    """Reprint the live text panel in place (no subprocess, no scroll spam)."""
    lines = [
        "================ LOCKE DRONE TELEMETRY ================",
        f" Status:        {status}",
        f" Speed:         {fps:.1f} FPS",
    ]
    if telem:
        lines += [
            f" Confidence:    {telem['confidence'] * 100:.1f}%",
            f" Dist target:   {fmt(telem['distance'])} cm",
            f" Est altitude:  {fmt(telem['altitude'])} cm",
            f" Offset X:      {telem['offset_x']:.0f} px",
            f" Head lock:     {telem['head']}",
        ]
    lines.append("=======================================================")
    # \033[H = cursor home, \033[J = clear below: overwrites the panel without a shell call.
    print("\033[H\033[J" + "\n".join(lines))


def pick_inference_size(native_size):
    """Use the size baked into the ncnn model, warning if it disagrees with config.

    Prevents the 'malloc(): invalid size' crash you get from feeding an ncnn export
    a resolution it wasn't built for.
    """
    if native_size and native_size != config.INFER_SIZE:
        print(f"[WARN] Model is baked at {native_size}px but config.INFER_SIZE is "
              f"{config.INFER_SIZE}px. Using {native_size}px to stay safe — run "
              f"export_model.py to re-bake at {config.INFER_SIZE}px.")
        return native_size
    return config.INFER_SIZE


def main():
    print("[INFO] Starting Locke Drone tracker...")
    camera = Camera(config.CAMERA_INDEX, config.FRAME_WIDTH, config.FRAME_HEIGHT)
    model, native_size = load_model(config.MODEL_DIR)
    infer_size = pick_inference_size(native_size)

    quit_hint = "Press 'q' in the window to quit." if config.SHOW_WINDOW else "Ctrl+C to quit."
    print(f"[INFO] Ready. Inference at {infer_size}px. {quit_hint}")

    lost_frames = 0
    frame_count, fps, fps_t0 = 0, 0.0, time.time()
    last_dashboard = 0.0

    try:
        while True:
            frame = camera.read()
            if frame is None:
                continue

            box = detect_person(model, frame, config.CONFIDENCE,
                                 infer_size, config.DEVICE, config.PERSON_CLASS)

            if box:
                lost_frames = 0
                telem = compute_telemetry(box, frame.shape[1], frame.shape[0])
                if config.SHOW_WINDOW:
                    draw_overlays(frame, box, telem["head"])
                status = "TRACKING TARGET - LOCK ACQUIRED"
            else:
                lost_frames += 1
                telem = None
                if lost_frames < config.LOST_TARGET_GRACE:
                    status = f"[HOLD] Target lost, filtering ({lost_frames}/{config.LOST_TARGET_GRACE})"
                else:
                    status = "[SEARCHING] No targets in frame"

            # Measure real FPS over wall-clock, refreshed once a second.
            frame_count += 1
            elapsed = time.time() - fps_t0
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count, fps_t0 = 0, time.time()

            # Throttle the text panel so printing doesn't eat the frame budget.
            now = time.time()
            if now - last_dashboard >= 1.0 / config.DASHBOARD_HZ:
                render_dashboard(status, fps, telem)
                last_dashboard = now

            if config.SHOW_WINDOW:
                cv2.imshow("Locke Drone Vision", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        print("\n[INFO] Shutting down...")
        camera.release()
        cv2.destroyAllWindows()
        print("[INFO] Tracker offline.")


if __name__ == "__main__":
    main()
