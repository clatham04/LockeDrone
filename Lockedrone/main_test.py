import cv2
import time
import torch

# Import custom components from your other project files
from camera import BackgroundCamera
from detector import HumanDetector
from telemetry import TelemetryCalculator

# ── CONFIGURATION & INITIALIZATION ──────────────────────────────────────────
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
FRAME_W, FRAME_H = 640, 480

print(f"Initializing Locke Drone Navigation Software Suite...")
cam = BackgroundCamera(src=0, width=FRAME_W, height=FRAME_H)
detector = HumanDetector(model_path="yolo11n.pt", device=DEVICE)
telemetry = TelemetryCalculator(known_width=50.0, known_height=170.0, focal_length=640.0, camera_tilt=15.0)

print(f"System Operational -> Hardware Cluster Mapping: {DEVICE.upper()}")
time.sleep(1.0)  # Allow sensor pipeline baseline calibration delay

fps_smooth = 30.0
prev_time = time.perf_counter()

# ── PRIMARY LOOP ─────────────────────────────────────────────────────────────
while True:
    ret, frame = cam.read()
    if not ret: 
        break

    # Performance Analytics Tracker (FPS calculation)
    now = time.perf_counter()
    fps_smooth = 0.1 * (1.0 / (now - prev_time)) + 0.9 * fps_smooth
    prev_time = now

    # Step 1: Detect Target
    target = detector.detect_primary_target(frame, conf_threshold=0.45, imgsz=320)

    if target:
        x1, y1, x2, y2, conf = target
        w, h = x2 - x1, y2 - y1
        cx, cy = x1 + (w // 2), y1 + (h // 2)

        # Step 2: Extract Telemetry Statistics
        dist_human = telemetry.get_distance_to_human(w, h)
        dist_ground = telemetry.get_distance_to_ground(dist_human, y2, frame_height=FRAME_H)

        # Step 3: Draw Tracking UI overlays
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        human_m = f"Dist to Human: {dist_human/100:.2f}m" if dist_human else "Dist: --"
        ground_m = f"Dist to Ground: {dist_ground/100:.2f}m" if dist_ground else "Alt: --"
        
        for i, text in enumerate([f"Target: Person ({conf:.0%})", human_m, ground_m]):
            cv2.putText(frame, text, (x1, y1 - 10 - (i * 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "🛰️ SCANNING FOR TARGET...", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    # Global Frame Diagnostic Banner
    cv2.putText(frame, f"FPS: {fps_smooth:.1f} ({DEVICE.upper()})", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    cv2.imshow("Locke Drone Core Feed (Modular Architecture)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"): 
        break

# Resource Disposal Sequence
cam.release()
cv2.destroyAllWindows()
print("System powered down smoothly.")