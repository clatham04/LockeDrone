import cv2

# ── Detection ────────────────────────────────────────────────────────────────
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def detect_person(frame):
    """Run HOG detection and return the best full-body bounding box, or None."""
    boxes, weights = hog.detectMultiScale(
        frame,
        winStride=(8, 8),
        padding=(8, 8),
        scale=1.03
    )
    if len(boxes) == 0:
        return None

    # Pick highest-confidence detection
    best_idx = weights.flatten().argmax()
    x, y, w, h = boxes[best_idx]

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
DETECT_EVERY_N   = 30      # Re-run HOG every N frames to confirm target
MIN_DETECT_CONF  = 0.4     # Minimum HOG weight to accept a detection
LOST_PATIENCE    = 45      # Frames to wait before resetting after tracker loss

tracker          = None
tracking         = False
bbox             = None
frame_count      = 0
lost_count       = 0


# ── Camera ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

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

    # ── Phase 1: Searching for a person ──────────────────────────────────────
    if not tracking:
        detected = detect_person(frame)

        if detected:
            bbox    = detected
            tracker = make_tracker()
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

            # Periodically re-run HOG to re-anchor the tracker
            if frame_count % DETECT_EVERY_N == 0:
                detected = detect_person(frame)
                if detected:
                    # HOG found someone — reinitialise tracker on fresh box
                    bbox    = detected
                    tracker = make_tracker()
                    tracker.init(frame, bbox)

        else:
            # Tracker failed this frame — be patient before giving up
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

            # Bounding box
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Centroid dot
            cv2.circle(display, (cx, cy), 6, (0, 0, 255), -1)

            # Corner accents (feels more like a targeting reticle)
            accent = 18
            thickness = 3
            color = (0, 255, 0)
            for px, py in [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]:
                dx = accent if px == x else -accent
                dy = accent if py == y else -accent
                cv2.line(display, (px, py), (px + dx, py), color, thickness)
                cv2.line(display, (px, py), (px, py + dy), color, thickness)

            # Status label
            status = "TRACKING" if lost_count == 0 else f"RECOVERING ({lost_count})"
            cv2.putText(display, status, (x, y - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Centroid coords (useful for drone control later)
            cv2.putText(display, f"center: ({cx}, {cy})", (x, y + h + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    cv2.imshow("Locke — Human Tracker", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('r'):
        print("Manual reset.")
        tracking, tracker, bbox, lost_count = False, None, None, 0

cap.release()
cv2.destroyAllWindows()