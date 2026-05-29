## human_lock code documentation ##

Locke Drone Autonomous Tracking System — Software Architecture Manual
This document provides a comprehensive breakdown of the core software components driving the Locke Drone autonomous vision, tracking, and telemetry calculation pipeline.

The software architecture is engineered to run seamlessly on both desktop environments (for prototyping) and edge computing hardware like the Raspberry Pi 4B (for deployment). It automatically adjusts its performance settings based on the host operating system.

1. System Dependency Installation
Before executing the core tracking software, your environment must be populated with the correct lightweight computer vision and machine learning frameworks.

Python Requirements Configuration (requirements.txt)
Save the following configuration into a file named requirements.txt in your project root folder:

Plaintext
# Core Computer Vision & Edge AI Architecture
ultralytics>=8.3.0
opencv-contrib-python
ncnn                   # Natively accelerates YOLO models on the Raspberry Pi 4 CPU

# NCNN Required Sub-Dependencies
portalocker
tqdm

# Video Handling & Math Utilities
av                     # Crucial dependency for Ultralytics video handling
numpy
Installation Steps
On Your Development PC (Windows)
Open PowerShell or your system terminal within your active virtual environment (.venv) and execute:

PowerShell
# 1. Activate your virtual environment if not already active
.venv\Scripts\Activate.ps1

# 2. Update and sync your packages cleanly
pip install -r requirements.txt
On the Raspberry Pi 4B (Linux)
Before installing the Python packages via pip, the underlying system libraries for matrix mathematical computations and open-source thread scheduling must be present on your Linux distribution. Run the following commands in the Pi's terminal:

Bash
# 1. Update system package repositories
sudo apt update && sudo apt upgrade -y

# 2. Install hardware-level BLAS and OpenMP packages required by NCNN
sudo apt install -y libopenblas-dev libgomp1

# 3. Activate your virtual environment and install dependencies
source .venv/bin/activate
pip install -r requirements.txt
2. Core Architectural Components
The tracking architecture is modularized into four distinct Python files. Each script isolates a specific physical or logical layer of the drone's runtime execution.

Module 1: camera.py (Hardware Capture Layer)
This script abstracts hardware interaction with the camera sensor. Its key innovation is an Automatic Operating System Switcher that protects resource allocation depending on whether the drone is bench-testing or flying.

Windows Operation (Bench-Testing Mode): Windows laptop webcams often experience thread-deadlocks when bombarded with high-speed parallel frame captures. On Windows, camera.py disables complex background multithreading and taps directly into sequential hardware frame collection using the stable cv2.CAP_DSHOW (DirectShow) video driver pipeline.

Linux Operation (Flight Mode): On the Raspberry Pi, the script automatically forks a dedicated, non-blocking background thread. It targets the native kernel-level video capture pipeline (cv2.CAP_V4L2) and restricts the internal image buffer size to exactly 1 frame (cv2.CAP_PROP_BUFFERSIZE = 1). This completely eliminates "lag buildup," ensuring the AI always evaluates the absolute newest physical frame.

Module 2: detector.py (Edge Artificial Intelligence Layer)
This file governs spatial object categorization. Instead of initializing heavy, processing-intensive deep learning networks, it interfaces directly with your pre-compiled, optimized NCNN model directory (yolo11n_ncnn_model).

NCNN is a high-performance neural network inference framework optimized explicitly for mobile platforms and ARM-based hardware architectures (like the Raspberry Pi CPU).

The script processes input frames down to a reduced pixel array envelope (imgsz=256), stripping away extra resolution overhead to maximize speed. It isolates the primary human entity in the field of view and outputs structural bounding box boundaries (x1, y1, x2, y2) alongside a fractional tracking certainty metric (confidence).

Module 3: telemetry.py (Spatial Translation Math)
This file acts as the drone's digital radar, translating 2D flat camera pixel data into true physical metric dimensions without requiring heavy stereo-vision sensors or active LiDAR modules.

Distance Translation: Using the mathematical properties of focal length ratios, the module calculates distance relative to an average human scale baseline (assuming a physical target width of 50cm and height of 170cm). If the pixel bounding box shrinks, the target is known to be moving away; if it expands, the target is approaching.

Altitude Extrapolation: Using the known geometric pitch angle of your physical drone camera casing (camera_tilt=15.0 degrees) and mapping the distance estimation alongside where the target's feet intersect the bottom horizontal raster boundary, the class applies trigonometry to output an approximate spatial relative altitude vector.

Module 4: human_lock.py (Unified Control & Telemetry Dashboard)
The main orchestration hub. It handles the central infinite runtime loop, pulls frames, drives detection commands, processes the structural math calculations, and controls the visual outputs.

Pacing Delay: Implements a microsecond time.sleep(0.005) loop throttle to prevent host CPU thread starvation, safeguarding device stability during sustained execution blocks.

Head Targeting Math: Translates body position data to establish a precise coordinate lock target on the human head. By taking the horizontal midpoint of the entity (target_center_x) and dropping down by exactly 10% of the calculated total body bounding frame height (y1 + box_height * 0.10), it maps a dynamic coordinate set. This ensures that whether you move near or far, a visual tracking lock (a filled red circle) sits consistently on the target's head.

Unified Live-Updating Dashboard: Rather than writing repetitive log dumps that scroll into a massive wall of text, the script monitors its loop sequences via a frames-per-second modulus counter (fps_counter % 10). Every ten loops, it systematically clears the command screen and prints a fixed-width, live-updating instrumentation panel revealing status, system calculation rates (FPS), target metrics, and spatial vectors.

Hysteresis Filter (Lost Target Hold): Protects flight stability from momentary signal drops (e.g. from rapid movement or motion blur). If the AI model loses its target detection lock for a split second, the script triggers a 5-frame grace tracking filter. It retains last-known coordinates smoothly rather than causing the drone to abruptly snap into an erratic searching behavior.