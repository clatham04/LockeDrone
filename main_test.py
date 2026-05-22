import cv2
import time
import threading

# ── Detection ────────────────────────────────────────────────────────────────
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def detect_person(frame):
    """Run HOG on a downscaled frame, return full-res bbox or None."""
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
    x, y, w, h = (int(v / scale_factor) for v in (x, y, w, h))

    pad_x = int(w * 0.10)
    pad_y = int(h * 0.15)
    x = max(0, x - pad_x)
    y = max(0, y - pad_y)
    w = min(frame.shape[1] - x, w + 2 * pad_x)
    h = min(frame.shape[0] - y, h + 2 * pad_y)

    return (x, y, w, h)


# ── Background detection thread ───────────────────────────────────────────────
class DetectionThread:
    """
    Runs HOG detection in a background thread so it never blocks the main loop.
    The main loop reads .result whenever it's ready.
    """
    def __init__(self):
        self.result     = None          # Latest detection result
        self.running    = False         # True while a detection is in progress
        self._lock      = threading.Lock()

    def request(self, frame):
        """Kick off a detection if one isn't already running."""
        if self.running:
            return                      # Don't queue — skip this frame
        self.running = True
        t = threading.Thread(target=self._run, args=(frame.copy(),), daemon=True)
        t.start()

    def _run(self, frame):
        result = detect_person(frame)
        with self._lock:
            self.result = result
        self.running = False

    def get_result(self):
        """Returns latest result and clears it (None if nothing new)."""
        with self._lock:
            r, self.result = self.result, None
        return r


# ── Tracker factory ───────────────────────────────────────────────────────────
def make_tracker():
    return cv2.TrackerCSRT_create()


# ── State ─────────────────────────────────────────────────────────────────────
DETECT_EVERY_N = 30        # How often to request a background re-detection
LOST_PATIENCE  = 45

tracker     = None
tracking    = False
bbox        = None
frame_count = 0
lost_count  = 0
detector    = DetectionThread()

# ── FPS ───────────────────────────────────────────────────────────────────────
fps          = 0.0
fps_alpha    = 0.1
prev_time    = time.perf_counter()
FPS_TARGET   = 30
FPS_COLOR_OK   = (0, 255, 0)
FPS_COLOR_WARN = (0, 165, 255)
FPS_COLOR_LOW  = (0, 0, 255)

# ── Camera ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)

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

    # ── FPS ───────────────────────────────────────────────────────────────────
    now       = time.perf_counter()
    delta     = now - prev_time
    prev_time = now
    instant   = 1.0 / delta if delta > 0 else fps
    fps       = fps_alpha * instant + (1 - fps_alpha) * fps

    # ── Phase 1: Scanning ─────────────────────────────────────────────────────
    if not tracking:
        # Keep requesting detection every frame while scanning
        detector.request(frame)
        result = detector.get_result()

        if result:
            bbox       = result
            tracker    = make_tracker()
            tracker.init(frame, bbox)
            tracking   = True
            lost_count = 0
            print("🔒 Locked on to person.")
        else:
            cv2.putText(display, "Scanning for person...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

    # ── Phase 2: Tracking ─────────────────────────────────────────────────────
    else:
        ok, tracked_box = tracker.update(frame)

        if ok:
            bbox       = tuple(int(v) for v in tracked_box)
            lost_count = 0

            # Request background re-detection periodically — non-blocking
            if frame_count % DETECT_EVERY_N == 0:
                detector.request(frame)

            # If a fresh detection came back, reinit tracker on it
            fresh = detector.get_result()
            if fresh:
                bbox    = fresh
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

        # ── Draw ─────────────────────────────────────────────────────────────
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