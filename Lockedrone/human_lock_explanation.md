### main_test.py markdown file ###


# Locke Drone System Architecture Documentation

This document provides a technical breakdown of the modular computer vision suite driving the **Locke Drone** project. The software is split into separate, specialized modules to maximize processing efficiency, decouple visual calculations from hardware constraints, and prepare the system for upstream integration with flight controllers.

---

## Architecture Overview

The system is split into four distinct files that work in an orchestration chain. By uncoupling frame grabbing from model inference, the script achieves zero I/O blocking, optimizing the frame processing pipeline for real-time edge processing.

```
                  ┌──────────────────────┐
                  │   BackgroundCamera   │  <--- Continuous non-blocking 
                  │     (camera.py)      │       hardware capture loop
                  └──────────┬───────────┘
                             │ (Latest Frame)
                             ▼
                  ┌──────────────────────┐
                  │      main_test.py    │  <--- Orchestration Hub
                  └────┬────────────┬────┘       & UI Rendering
                       │            │
  (Inference Requests) │            │ (Bounding Box Specs)
                       ▼            ▼
 ┌──────────────────────┐  ┌──────────────────────┐
 │     HumanDetector    │  │ TelemetryCalculator  │
 │     (detector.py)    │  │    (telemetry.py)    │
 └──────────────────────┘  └──────────────────────┘
  Runs YOLOv11 on GPU       Computes physical human
  to isolate target info    & altitude ranges

```

---

## 1. `camera.py` (Asynchronous Video Processing)

### Purpose

Standard opencv camera reads (`cv2.VideoCapture().read()`) are blocking operations. The main script must pause and wait for the webcam hardware to physically register a frame, trapping your execution loop at the camera's baseline capture capability (typically 30 FPS).

`camera.py` fixes this by pushing the camera pipeline into a dedicated background execution thread using Python's `threading` and `lock` primitives.

### Code Mechanics

* **`threading.Thread`**: Spins up a background daemon worker loop that continually drains frames out of the video buffer as fast as the operating system permits.
* **`threading.Lock()`**: Prevents memory corruption or screen tearing. The frame is locked briefly while being copied, ensuring the main thread never reads a partially written frame matrix.
* **`read()`**: Returns an instantaneous copy of the freshest frame stored in memory, allowing your core loop to cycle at maximum GPU calculation speeds.

---

## 2. `detector.py` (Object Detection Pipeline)

### Purpose

This module handles object identification and filtering. It initializes the neural network model and isolates human coordinates while stripping away unneeded tracking parameters.

### Code Mechanics

* **Ultralytics YOLOv11 Nano (`yolo11n.pt`)**: Uses a compact structure built for edge performance.
* **Hardware Allocation Matrix**: Detects if your host machine has a CUDA-enabled graphic card environment (like your laptop's RTX 3070 Ti) and pushes the entire model layer matrix onto GPU memory space.
* **`detect_primary_target()`**:
* **`classes=[0]`**: Restricts the detection window exclusively to the `person` index inside the COCO object training map.
* **`imgsz=320`**: Scales raw frame textures down to an optimized resolution inside the network layers, slashing mathematical calculations by roughly 75% compared to native 640x480 pipelines.
* **`boxes.conf.argmax()`**: Dynamically filters out background noise. If multiple people appear, it calculates the highest confidence array, returning a singular target bounding bracket `(x1, y1, x2, y2)` to prevent tracking confusion.



---

## 3. `telemetry.py` (Visual Spatial Mathematics)

### Purpose

Drones require physical metric boundaries to manage follow speeds. This component abstracts standard video pixels into actual real-world distances.

### Code Mechanics

* **`get_distance_to_human()`**: Uses the **Pinhole Camera Model** (Monocular Focal Calibration Formula). By comparing the known physical shoulder width (50 cm) and average height (170 cm) against the pixel dimensions (w, h) inside the bounding box, it determines range depth without requiring a stereoscopic or LiDAR system.
* **`get_distance_to_ground()`**: Computes instantaneous aircraft altitude estimation.
* Measures the vertical pixel delta from the screen horizontal horizon line (480 / 2) down to the target's base feet tracking boundaries (y_2).
* Merges the camera lens manual tilt offset (`CAMERA_TILT_DEG`) with the pixel angle offset to compute a total vector projection line.
* Uses a trigonometric sine function (`math.sin()`) against the hypotenuse distance vector to gauge distance to the ground plane, filtering out domain errors below parallel horizons.



---

## 4. `main_test.py` (The Central Orchestrator)

### Purpose

The execution manager. It initializes all subsystems, provisions memory parameters, maps data loops, and renders output diagnostics to the dashboard monitor.

### Key Functional Loops

1. **Drains Sensors**: Polls the asynchronous camera module for the freshest image copy.
2. **Computes Vision**: Passes the image matrix directly over to the GPU model thread to look for a person.
3. **Translates Spatial Metrics**: If a person is present, it extracts pixel dimensions and hands them over to the telemetry engine to obtain tracking ranges.
4. **Draws Heads-Up Display**: Layers vector geometry paths, targeting center reticles, and numeric diagnostic indicators onto the display array.
5. **Calculates System Efficiency**: Logs system latency across execution cycles to compute a true rolling FPS calculation.

---

## Project Customization Vectors

When preparing this modular script to be deployed on custom physical drone hardware, adjust these configuration parameters inside `main_test.py`:

| Constant | System impact |
| --- | --- |
| `DEVICE` | Change `"cuda"` to `"cpu"` if moving to an unaccelerated CPU platform like a Raspberry Pi. |
| `CAMERA_TILT_DEG` | Match the exact physical downward angle (in degrees) of your drone's camera frame mount. |
| `KNOWN_WIDTH_CM` | Calibrate this to the specific shoulder width of your primary flight target for increased distance precision. |
| `imgsz=320` | Lower this inside `detector.py` to `256` if you require higher frame speeds on lightweight companion processors. |