# Locke Drone — Human Tracker Code Explained

> `src/vision/human_detector.py` — real-time human detection and tracking using OpenCV

---

## Overview

The tracker works in two distinct phases that hand off to each other automatically:

1. **Scanning** — HOG detector searches the frame for a person
2. **Tracking** — once a person is found, CSRT takes over and follows them frame-by-frame

HOG detection is expensive (~150ms per call), so it runs on a **background thread** and never blocks the camera loop. The camera loop itself only ever runs the lightweight CSRT tracker update (~5ms), which is what keeps FPS high.

---

## Imports

```python
import cv2
import time
import threading
```

| Import | Purpose |
|---|---|
| `cv2` | OpenCV — camera access, HOG detection, CSRT tracker, drawing |
| `time` | `time.perf_counter()` for high-precision FPS measurement |
| `threading` | Runs HOG detection in the background without blocking the frame loop |

---

## HOG People Detector

```python
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
```

**HOG** (Histogram of Oriented Gradients) is a classical computer vision algorithm that detects humans by analysing the pattern of edge directions across a sliding window. OpenCV ships with a pre-trained SVM model for this, so no external model files are needed.

### `detect_person(frame)`

```python
def detect_person(frame):
    scale_factor = 0.5
    small = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor)

    boxes, weights = hog.detectMultiScale(small, winStride=(8,8), padding=(8,8), scale=1.05)
```

| Step | What it does |
|---|---|
| **Downscale to 50%** | HOG runs on a half-size frame — ~4× faster, same detection quality at normal distances |
| **`detectMultiScale`** | Slides the detector window across the image at multiple scales to catch people at different distances |
| **`winStride=(8,8)`** | How many pixels the window moves each step — smaller = more thorough, slower |
| **`padding=(8,8)`** | Extra pixels added around the detection window edge |
| **`scale=1.05`** | Image pyramid scale factor — 1.05 builds many layers (thorough), 1.1+ is faster but misses more |

After detection, coordinates are scaled back up to full resolution, then padded:

```python
pad_x = int(w * 0.10)
pad_y = int(h * 0.15)
```

HOG tends to clip the top of the head and the feet. The 10% horizontal and 15% vertical padding corrects for this so the bounding box covers the full body.

---

## Background Detection Thread

This is the key architectural decision that keeps FPS high.

```python
class DetectionThread:
    def request(self, frame):
        if self.running:
            return          # Already busy — skip, don't queue
        self.running = True
        t = threading.Thread(target=self._run, args=(frame.copy(),), daemon=True)
        t.start()
```

### How it works

```
Main loop (30+ FPS)          Background thread
─────────────────            ─────────────────
read frame        ──────►    HOG detection (~150ms)
tracker.update()             result stored in self.result
draw overlay
check get_result() ◄──────   result ready
reinit tracker if needed
```

- `request()` — fires off a detection on a **copy** of the frame. If one is already running, it skips silently rather than queuing (queuing would cause the result to be stale by the time it arrives).
- `_run()` — the actual HOG call. Stores the result under a `threading.Lock` so it's safe to read from the main thread.
- `get_result()` — called from the main loop each frame. Returns the latest result and clears it, so each detection is consumed exactly once.

---

## State Variables

```python
DETECT_EVERY_N = 30    # Re-run HOG every N frames while tracking
LOST_PATIENCE  = 45    # Frames to wait before giving up on a lost target
```

| Variable | Purpose |
|---|---|
| `tracker` | The active CSRT tracker instance, or `None` when scanning |
| `tracking` | Boolean flag — `False` = scanning phase, `True` = tracking phase |
| `bbox` | The current bounding box `(x, y, w, h)` of the tracked person |
| `frame_count` | Total frames processed — used to time periodic HOG re-runs |
| `lost_count` | Consecutive frames where the tracker has failed — patience counter |

---

## Main Loop

### FPS Calculation

```python
now       = time.perf_counter()
delta     = now - prev_time
prev_time = now
instant   = 1.0 / delta if delta > 0 else fps
fps       = fps_alpha * instant + (1 - fps_alpha) * fps
```

Raw per-frame FPS (`instant`) is smoothed with an **exponential moving average**:

```
fps = 0.1 × instant + 0.9 × previous_fps
```

`fps_alpha = 0.1` means the display reacts slowly to changes — readable, not jittery.

---

### Phase 1 — Scanning

```python
if not tracking:
    detector.request(frame)
    result = detector.get_result()

    if result:
        tracker = make_tracker()
        tracker.init(frame, bbox)
        tracking = True
```

Every frame, a HOG detection is requested. Most frames get `None` back (the previous detection is still running). When a result arrives, CSRT is initialised on the detected box and the state switches to tracking.

---

### Phase 2 — Tracking

```python
ok, tracked_box = tracker.update(frame)
```

`tracker.update()` is the core of the tracking loop. CSRT (Channel and Spatial Reliability Tracking) maintains an internal appearance model of the target and searches for it in the new frame. It returns:

- `ok = True` + `tracked_box` — target found, here's the new position
- `ok = False` — target lost this frame

**When tracking succeeds:**

```python
if frame_count % DETECT_EVERY_N == 0:
    detector.request(frame)     # Non-blocking HOG re-run in background

fresh = detector.get_result()
if fresh:
    tracker = make_tracker()    # Reinit on fresh detection to prevent drift
    tracker.init(frame, bbox)
```

Every 30 frames, HOG runs again in the background to re-anchor the tracker. If CSRT drifts slightly over time, the next HOG result snaps it back. This reinit is what was previously blocking the main thread — now it's invisible to the frame loop.

**When tracking fails:**

```python
lost_count += 1
if lost_count >= LOST_PATIENCE:
    tracking = False    # Back to scanning
```

A single failed frame doesn't reset the tracker — it waits 45 frames. This handles brief occlusion (person walks behind something), sudden motion blur, or lighting changes without abandoning the target unnecessarily.

---

### Drawing

| Element | Code | Purpose |
|---|---|---|
| Bounding box | `cv2.rectangle` | Full-body outline in green |
| Centroid dot | `cv2.circle` | Red dot at the body's centre point |
| Corner accents | `cv2.line` × 8 | Targeting reticle feel — communicates a stable lock |
| Status label | `cv2.putText` | `TRACKING` or `RECOVERING (N)` above the box |
| Centroid coords | `cv2.putText` | `center: (cx, cy)` — consumed by `tracker.py` for drone navigation |

The centroid `(cx, cy)` is the key output for the drone: it tells `flight_manager.py` which direction to move and by how much to keep the person centred in frame.

---

### FPS Overlay

```python
if fps >= FPS_TARGET:          # ≥ 30  → green
    fps_color = FPS_COLOR_OK
elif fps >= FPS_TARGET - 5:    # 25–29 → orange
    fps_color = FPS_COLOR_WARN
else:                          # < 25  → red
    fps_color = FPS_COLOR_LOW
```

A dark filled rectangle is drawn behind the FPS text so it remains readable over any background — white walls, dark rooms, or outdoor scenes.

---

### Keyboard Controls

| Key | Action |
|---|---|
| `q` | Quit and release the camera |
| `r` | Force-reset the tracker — useful if it locks on to the wrong person |

---

## Performance Summary

| Operation | Cost per frame | Runs on |
|---|---|---|
| `cap.read()` | ~1ms | Main thread |
| `tracker.update()` | ~5–10ms | Main thread |
| Drawing | ~1ms | Main thread |
| HOG `detectMultiScale` | ~100–200ms | Background thread |
| CSRT reinit | ~20ms | Background thread |

The main loop only pays for the first three rows — roughly 7–12ms total — which is comfortably above 30 FPS. HOG and reinit happen concurrently without the camera loop ever waiting on them.

---

## Data Flow Into the Drone System

```
human_detector.py
      │
      │  bbox (x, y, w, h)
      │  centroid (cx, cy)
      ▼
  tracker.py          ← uses centroid to calculate directional error
      │
      │  direction vector + magnitude
      ▼
distance_estimator.py ← uses bbox height to estimate real-world distance
      │
      │  distance + direction
      ▼
flight_manager.py     ← translates to drone movement commands
      │
      ▼
  controller.py       ← sends commands to the drone hardware
```