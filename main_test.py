import cv2
import time

# ── Detection ────────────────────────────────────────────────────────────────
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def detect_person(frame):
    """Run HOG detection and return the best full-body bounding box, or None."""
    # Downscale for detection only — much faster, then scale coords back up
    scale_factor = 0.5
    small = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor)

    boxes, weights = hog.detectMultiScale(
        small,
        winStride=(8, 8),
        padding=(8, 8),
        scale=1.05
    )
    if len(boxes) == 0:
        return None

    best_idx = weights.flatten().argmax()
    x, y, w, h = boxes[best_idx]

    # Scale coords back to full resolution
    x, y, w, h = (int(v / scale_factor) for v in (x, y, w, h))

    # Expand box to capture full body (HOG tends to clip head/feet)
    pad_x = int(w * 0.1)
    pad_y = int(h * 0.15)
    x = max(0, x - pad_x)
    y = max(0, y - pad_y)
    w = min(frame.shape[1] - x, w + 2 * pad_x)
    h = min(frame.shape[0] - y, h + 2 * pad_y)

    return (x, y, w, h)


# ── Tracker factory ───────────────────────────────────────────────────────────
def make_tracker():
    """CSRT tracker — best accuracy for human bodies, handles occlusion well."""
    return cv2.TrackerCSRT_create()


# ── State ─────────────────────────────────────────────────────────────────────
DETECT_EVERY_N  = 30
MIN_DETECT_CONF = 0.4
LOST_PATIENCE   = 45

tracker    = None
tracking   = False
bbox       = None
frame_count = 0
lost_count  = 0

# ── FPS tracking ──────────────────────────────────────────────────────────────
fps              = 0.0
fps_alpha        = 0.1          # Smoothing factor (lower = smoother, slower to react)
prev_time        = time.perf_counter()
FPS_TARGET       = 30
FPS_COLOR_OK     = (0, 255, 0)   # Green  — at or above target
FPS_COLOR_WARN   = (0, 165, 255) # Orange — close to target (within 5)
FPS_COLOR_LOW    = (0, 0, 255)   # Red    — below target


# ── Camera ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)   # Request 60fps from camera if supported

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'q' to quit | 'r' to force reset tracker")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    frame_count += 1
    display = frame.copy()

    # ── FPS calculation (exponential moving average) ──────────────────────────
    now       = time.perf_counter()
    delta     = now - prev_time
    prev_time = now
    instant   = 1.0 / delta if delta > 0 else fps
    fps       = fps_alpha * instant + (1 - fps_alpha) * fps   # Smooth

    # ── Phase 1: Searching for a person ──────────────────────────────────────
    if not tracking:
        detected = detect_person(frame)

        if detected:
            bbox       = detected
            tracker    = make_tracker()
            tracker.init(frame, bbox)
            tracking   = True
            lost_count = 0
            print("🔒 Locked on to person.")
        else:
            cv2.putText(display, "Scanning for person...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    # ── Phase 2: Actively tracking ────────────────────────────────────────────
    else:
        ok, tracked_box = tracker.update(frame)

        if ok:
            bbox       = tuple(int(v) for v in tracked_box)
            lost_count = 0

            if frame_count % DETECT_EVERY_N == 0:
                detected = detect_person(frame)
                if detected:
                    bbox    = detected
                    tracker = make_tracker()
                    tracker.init(frame, bbox)

        else:
            lost_count += 1
            if lost_count >= LOST_PATIENCE:
                print("⚠️  Lost target. Scanning...")
                tracking   = False
                tracker    = None
                bbox       = None
                lost_count = 0

        # ── Draw bounding box & reticle ───────────────────────────────────────
        if bbox:
            x, y, w, h = bbox
            cx, cy     = x + w // 2, y + h // 2

            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)

            accent, thickness, color = 18, 3, (0, 255, 0)
            for px, py in [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]:
                dx = accent if px == x else -accent
                dy = accent if py == y else -accent
                cv2.line(display, (px, py), (px + dx, py), color, thickness)
                cv2.line(display, (px, py), (px, py + dy), color, thickness)

            status = "TRACKING" if lost_count == 0 else f"RECOVERING ({lost_count})"
            cv2.putText(display, status, (x, y - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(display, f"center: ({cx}, {cy})", (x, y + h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # ── FPS overlay ───────────────────────────────────────────────────────────
    if fps >= FPS_TARGET:
        fps_color = FPS_COLOR_OK
    elif fps >= FPS_TARGET - 5:
        fps_color = FPS_COLOR_WARN
    else:
        fps_color = FPS_COLOR_LOW

    fps_text = f"FPS: {fps:.1f}"

    # Dark background pill behind FPS text for readability over any background
    (tw, th), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    cv2.rectangle(display, (8, 8), (18 + tw, 18 + th + 6), (0, 0, 0), -1)
    cv2.putText(display, fps_text, (13, 13 + th),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, fps_color, 2)

    cv2.imshow("Locke — Human Tracker", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('r'):
        print("Manual reset.")
        tracking, tracker, bbox, lost_count = False, None, None, 0

cap.release()
cv2.destroyAllWindows()