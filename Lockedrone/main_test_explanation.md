# Locke Drone — Human Tracker
> `src/vision/human_detector.py`

---

## What the code does in one sentence

It watches the webcam, finds a person using HOG detection, then hands off to a CSRT tracker that follows them frame-by-frame — without ever letting detection slow down the camera loop.

---

## Imports

```python
import cv2
import time
import threading
```

| Import | Why it's needed |
|---|---|
| `cv2` | OpenCV — camera, HOG detector, CSRT tracker, drawing |
| `time` | `perf_counter()` for precise FPS measurement |
| `threading` | Runs HOG in the background so it doesn't freeze the frame loop |

---

## Part 1 — HOG People Detector

```python
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
```

**HOG** (Histogram of Oriented Gradients) detects humans by analysing edge patterns across a sliding window. OpenCV ships a pre-trained model built in — no downloads or external files needed.

### `detect_person(frame)`

```python
def detect_person(frame):
    small = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
    boxes, weights = hog.detectMultiScale(small, winStride=(8,8), padding=(8,8), scale=1.05)
```

| Step | What it does |
|---|---|
| Resize to 50% | HOG runs on a half-size frame — ~4× faster, no meaningful accuracy loss at normal distances |
| `detectMultiScale` | Slides a detection window across the image at multiple zoom levels to catch people at different distances |
| `winStride=(8,8)` | How many pixels the window jumps each step — smaller = more thorough but slower |
| `scale=1.05` | How many zoom levels to check — 1.05 is thorough; 1.1 is faster but misses more |

After detection, coordinates are doubled back to full resolution, then padded:

```python
px, py = int(w * 0.10), int(h * 0.15)
```

HOG naturally clips the head and feet. The 10% horizontal and 15% vertical padding corrects for this so the box covers the whole body.

---

## Part 2 — Background Detection Thread

This is the most important piece of the architecture. HOG takes ~150ms per call — if it ran on the main thread it would cap the whole system at ~7 FPS. `DetectionThread` fixes this by running HOG on a separate thread so the camera loop never waits on it.

```python
class DetectionThread:
    def request(self, frame):
        if self._running:
            return   # Already busy — skip, don't queue
        self._running = True
        threading.Thread(target=self._run, args=(frame.copy(),), daemon=True).start()
```

### How the two threads interact

```
Main loop (30+ FPS)              Background thread
───────────────────              ─────────────────
read frame          ──────────►  HOG detection (~150ms)
tracker.update()                 result stored safely
draw overlay
check get_result()  ◄──────────  result ready
reinit tracker if fresh result
```

- **`request()`** — fires HOG on a copy of the frame. If one is already running it skips rather than queuing, keeping results fresh not stale.
- **`_run()`** — does the actual HOG call and stores the result under a lock so the main thread can safely read it.
- **`get_result()`** — called every frame from the main loop. Returns the result and clears it so each detection is used exactly once.

---

## Part 3 — Constants & State

```python
DETECT_EVERY_N = 30   # Re-run HOG every N frames while tracking
LOST_PATIENCE  = 45   # Frames of failure before returning to scan

FPS_COLORS = [(FPS_TARGET, green), (FPS_TARGET-5, orange), (0, red)]
```

| Variable | Purpose |
|---|---|
| `tracker` | Active CSRT tracker instance, or `None` when scanning |
| `tracking` | `False` = scanning phase, `True` = tracking phase |
| `bbox` | Current bounding box `(x, y, w, h)` of the tracked person |
| `frame_count` | Total frames — used to time the periodic HOG re-runs |
| `lost_count` | Consecutive frames where CSRT has failed — patience counter |

`FPS_COLORS` is a lookup table — the FPS overlay walks down the list and picks the first colour whose threshold the current FPS meets. Green ≥ 30, orange ≥ 25, red below that.

---

## Part 4 — Main Loop

### FPS Calculation

```python
fps = 0.1 * (1.0 / (now - prev_time)) + 0.9 * fps
```

This is an **exponential moving average**. Each frame, 10% of the new instant reading is blended in and 90% of the previous smoothed value is kept. The result reacts to real changes but stays readable — not jittery.

---

### Phase 1 — Scanning

```python
if not tracking:
    detector.request(frame)
    result = detector.get_result()

    if result:
        tracker = make_tracker()
        tracker.init(frame, bbox)
        tracking, lost_count = True, 0
```

Every frame, a HOG detection is requested. Most frames get `None` back — the previous detection is still running. When a result finally arrives, CSRT is initialised on the detected box and the state flips to tracking.

---

### Phase 2 — Tracking

```python
ok, tracked_box = tracker.update(frame)
```

`tracker.update()` is the core of the loop. CSRT builds an internal appearance model of the target and searches for it in each new frame. It's fast (~5ms) and handles partial occlusion well — which is why it's doing the per-frame work instead of HOG.

**If tracking succeeds (`ok = True`):**

```python
if frame_count % DETECT_EVERY_N == 0:
    detector.request(frame)   # Non-blocking — fires in background

fresh = detector.get_result()
if fresh:
    tracker = make_tracker()
    tracker.init(frame, fresh)  # Re-anchor to HOG result to prevent drift
```

Every 30 frames, HOG runs again in the background to re-anchor the tracker. CSRT can drift slightly over long tracking sessions — this periodic correction snaps it back without interrupting the frame loop.

**If tracking fails (`ok = False`):**

```python
lost_count += 1
if lost_count >= LOST_PATIENCE:
    tracker, tracking, bbox, lost_count = None, False, None, 0
```

One failed frame doesn't abandon the target. The 45-frame patience buffer handles brief occlusion, sudden motion blur, or lighting changes without jumping back to scan mode unnecessarily.

---

### Drawing

| Element | Colour | Purpose |
|---|---|---|
| Bounding box | Green | Full-body outline |
| Centroid dot | Red | Exact centre of the tracked person |
| Corner reticle | Green | 4 L-shaped accents — communicates a stable lock visually |
| Status label | Green | `TRACKING` or `RECOVERING (N)` |
| Centroid coords | Grey | `center: (cx, cy)` — the key output for drone navigation |

The centroid `(cx, cy)` is what flows downstream into `tracker.py` to tell the drone which direction to move and by how much to keep the person centred in frame.

---

### Keyboard Controls

| Key | Action |
|---|---|
| `q` | Quit and release the camera |
| `r` | Force-reset — useful if the tracker locks onto the wrong person |

---

## Performance at a Glance

| Operation | Time | Thread |
|---|---|---|
| `cap.read()` | ~1ms | Main |
| `tracker.update()` | ~5ms | Main |
| Drawing | ~1ms | Main |
| **Total main loop** | **~7ms → 30+ FPS** | |
| HOG `detectMultiScale` | ~150ms | Background |
| CSRT reinit | ~20ms | Background |

The main loop only ever pays for the top three rows. HOG and reinit run concurrently and invisibly.

---

## How This Fits Into the Drone

```
human_detector.py
    │  bbox (x, y, w, h)
    │  centroid (cx, cy)
    ▼
tracker.py          → calculates directional error from centroid
    ▼
distance_estimator.py → estimates real-world distance from bbox height
    ▼
flight_manager.py   → converts error + distance into movement commands
    ▼
controller.py       → sends commands to the drone hardware
```