import cv2
import time
import threading

# ── HOG People Detector ───────────────────────────────────────────────────────
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def detect_person(frame):
    """Run HOG on a downscaled frame, return full-res bbox or None."""
    small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    boxes, weights = hog.detectMultiScale(small, winStride=(8,8), padding=(8,8), scale=1.05)

    if len(boxes) == 0:
        return None

    x, y, w, h = boxes[weights.flatten().argmax()]
    x, y, w, h = (v * 2 for v in (x, y, w, h))  # Scale back to full resolution

    # Pad to cover full body (HOG clips head and feet)
    px, py = int(w * 0.10), int(h * 0.15)
    x = max(0, x - px)
    y = max(0, y - py)
    w = min(frame.shape[1] - x, w + 2 * px)
    h = min(frame.shape[0] - y, h + 2 * py)

    return x, y, w, h


# ── Background Detection Thread ───────────────────────────────────────────────
class DetectionThread:
    """Runs HOG in a background thread so it never blocks the main loop."""

    def __init__(self):
        self._result  = None
        self._running = False
        self._lock    = threading.Lock()

    def request(self, frame):
        """Start a detection if one isn't already running."""
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._run, args=(frame.copy(),), daemon=True).start()

    def _run(self, frame):
        result = detect_person(frame)
        with self._lock:
            self._result = result
        self._running = False

    def get_result(self):
        """Return the latest detection and clear it. Returns None if not ready."""
        with self._lock:
            result, self._result = self._result, None
        return result


# ── Tracker ───────────────────────────────────────────────────────────────────
def make_tracker():
    return cv2.TrackerCSRT_create()


# ── Constants ─────────────────────────────────────────────────────────────────
DETECT_EVERY_N = 30   # Frames between background HOG re-runs while tracking
LOST_PATIENCE  = 45   # Frames of tracker failure before returning to scan

FPS_TARGET     = 30
FPS_COLORS     = [(FPS_TARGET, (0,255,0)), (FPS_TARGET-5, (0,165,255)), (0, (0,0,255))]

# ── State ─────────────────────────────────────────────────────────────────────
tracker     = None
tracking    = False
bbox        = None
frame_count = 0
lost_count  = 0
detector    = DetectionThread()
fps         = 0.0
prev_time   = time.perf_counter()

# ── Camera ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS,          60)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Press 'q' to quit | 'r' to reset tracker")

# ── Main Loop ─────────────────────────────────────────────────────────────────
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    frame_count += 1
    display = frame.copy()

    # FPS — exponential moving average
    now       = time.perf_counter()
    fps       = 0.1 * (1.0 / (now - prev_time)) + 0.9 * fps
    prev_time = now

    # ── Phase 1: Scanning ─────────────────────────────────────────────────────
    if not tracking:
        detector.request(frame)
        result = detector.get_result()

        if result:
            bbox    = result
            tracker = make_tracker()
            tracker.init(frame, bbox)
            tracking, lost_count = True, 0
            print("🔒 Locked on to person.")
        else:
            cv2.putText(display, "Scanning for person...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    # ── Phase 2: Tracking ─────────────────────────────────────────────────────
    else:
        ok, tracked_box = tracker.update(frame)

        if ok:
            bbox, lost_count = tuple(int(v) for v in tracked_box), 0

            if frame_count % DETECT_EVERY_N == 0:
                detector.request(frame)

            fresh = detector.get_result()
            if fresh:
                bbox    = fresh
                tracker = make_tracker()
                tracker.init(frame, bbox)
        else:
            lost_count += 1
            if lost_count >= LOST_PATIENCE:
                print("⚠️  Lost target. Scanning...")
                tracker, tracking, bbox, lost_count = None, False, None, 0

        # Draw
        if bbox:
            x, y, w, h = bbox
            cx, cy = x + w // 2, y + h // 2

            cv2.rectangle(display, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(display, (cx, cy), 6, (0,0,255), -1)

            # Corner accent reticle
            a = 18
            for (x1,y1), (x2,y2), (x3,y3) in [
                ((x,y),   (x+a,y),   (x,y+a)),
                ((x+w,y), (x+w-a,y), (x+w,y+a)),
                ((x,y+h), (x+a,y+h), (x,y+h-a)),
                ((x+w,y+h),(x+w-a,y+h),(x+w,y+h-a)),
            ]:
                cv2.line(display, (x1,y1), (x2,y2), (0,255,0), 3)
                cv2.line(display, (x1,y1), (x3,y3), (0,255,0), 3)

            status = "TRACKING" if lost_count == 0 else f"RECOVERING ({lost_count})"
            cv2.putText(display, status,            (x, y-12),    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(display, f"center: ({cx},{cy})", (x, y+h+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

    # ── FPS Overlay ───────────────────────────────────────────────────────────
    fps_color = next(c for threshold, c in FPS_COLORS if fps >= threshold)
    fps_text  = f"FPS: {fps:.1f}"
    (tw, th), _ = cv2.getTextSize(fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    cv2.rectangle(display, (8, 8), (18+tw, 18+th+6), (0,0,0), -1)
    cv2.putText(display, fps_text, (13, 13+th), cv2.FONT_HERSHEY_SIMPLEX, 0.65, fps_color, 2)

    cv2.imshow("Locke — Human Tracker", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('r'):
        print("Manual reset.")
        tracker, tracking, bbox, lost_count = None, False, None, 0

cap.release()
cv2.destroyAllWindows()