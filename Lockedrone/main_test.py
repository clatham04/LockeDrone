import cv2
import time
import math
import torch
import threading
from ultralytics import YOLO

# ── 1. Asynchronous Camera Thread ─────────────────────────────────────────────
class BackgroundCamera:
    """Reads webcam frames in a separate thread to eliminate I/O blocking."""
    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.ret, self.frame = self.cap.read()
        self.running = True
        self.lock = threading.Lock()
        
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            time.sleep(0.01)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.ret else None

    def release(self):
        self.running = False
        self.cap.release()

# ── 2. Forced CPU Initialization ──────────────────────────────────────────────
model = YOLO("yolo11n.pt")
device = "cpu"  # Explicitly forcing CPU execution to test performance difference

print(f"Locke Drone Status -> Engine: {device.upper()} (Forced Benchmarking Mode)")

# ── 3. Calibration Constants ──────────────────────────────────────────────────
KNOWN_WIDTH_CM  = 50.0   
KNOWN_HEIGHT_CM = 170.0  
FOCAL_LENGTH_PX = 640.0  
CAMERA_TILT_DEG = 15.0   

# Start the non-blocking camera stream
cam = BackgroundCamera(0)
time.sleep(1.0) 

fps_smooth = 30.0
prev_time = time.perf_counter()

while True:
    ret, frame = cam.read()
    if not ret: break

    # Performance Timers 
    now = time.perf_counter()
    fps_smooth = 0.1 * (1.0 / (now - prev_time)) + 0.9 * fps_smooth
    prev_time = now

    # Run YOLOv11 strictly on the CPU
    results = model(frame, conf=0.45, classes=[0], verbose=False, imgsz=320, device="cpu")
    boxes = results[0].boxes

    if len(boxes) > 0:
        best_idx = int(boxes.conf.argmax())
        x1, y1, x2, y2 = map(int, boxes.xyxy[best_idx])
        conf = float(boxes.conf[best_idx])
        w, h = x2 - x1, y2 - y1
        cx, cy = x1 + w // 2, y1 + h // 2

        # ── Distance to Human ─────────────────────────────────────────────────
        d_width = (KNOWN_WIDTH_CM * FOCAL_LENGTH_PX) / w if w > 0 else None
        d_height = (KNOWN_HEIGHT_CM * FOCAL_LENGTH_PX) / h if h > 0 else None
        estimates = [d for d in [d_width, d_height] if d is not None]
        dist_human = sum(estimates) / len(estimates) if estimates else None

        # ── Corrected Distance to Ground (Altitude) ───────────────────────────
        dist_ground = None
        if dist_human:
            px_from_center = (y1 + h) - (480 / 2) 
            angle_offset = (px_from_center / 240.0) * (45.0 / 2.0) 
            total_angle = CAMERA_TILT_DEG + angle_offset
            
            if total_angle > 0 and total_angle < 85:
                dist_ground = dist_human * math.sin(math.radians(total_angle))

        # ── UI Overlays (Changed color tuples back to green: (0, 255, 0)) ──────
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        human_m = f"Dist to Human: {dist_human/100:.2f}m" if dist_human else "Dist: --"
        ground_m = f"Dist to Ground: {dist_ground/100:.2f}m" if dist_ground else "Alt: --"
        
        for i, text in enumerate([f"Target: Person ({conf:.0%})", human_m, ground_m]):
            cv2.putText(frame, text, (x1, y1 - 10 - (i * 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    else:
        cv2.putText(frame, "🛰️ SCANNING FOR TARGET...", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

    # Left the overall system FPS benchmark marker yellow/red for interface clarity
    cv2.putText(frame, f"FPS: {fps_smooth:.1f} (CPU)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    cv2.imshow("Locke Drone Core Feed (CPU TESTING)", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"): break

cam.release()
cv2.destroyAllWindows()