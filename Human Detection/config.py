"""Locke Drone — all tunable settings in ONE place.

Change values here, not inside the other files. This is the file to open when you
want to tweak behaviour.

NOTE: after changing INFER_SIZE, re-run `python export_model.py` so the ncnn model
is re-baked to match. ncnn models have their input size compiled in.
"""

# --- Camera ---
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- Model & detection ---
MODEL_DIR = "yolo11n_ncnn_model"   # ncnn export folder, or "yolo11n.pt" for desktop testing
INFER_SIZE = 256                   # input resolution; smaller = faster on the Pi (must match the export)
CONFIDENCE = 0.30                  # minimum detection confidence, 0.0 - 1.0
PERSON_CLASS = 0                   # COCO class id for "person"
DEVICE = "cpu"

# --- Telemetry (real-world assumptions, centimetres / degrees) ---
KNOWN_WIDTH_CM = 50.0              # average shoulder width of a person
KNOWN_HEIGHT_CM = 170.0           # average person height
FOCAL_LENGTH_PX = 640.0           # camera focal length in pixels
CAMERA_TILT_DEG = 15.0           # downward tilt of the drone camera

# --- Display & behaviour ---
SHOW_WINDOW = True                 # set False for a headless drone run (a few FPS faster)
DASHBOARD_HZ = 4                   # how many times per second to refresh the text panel
LOST_TARGET_GRACE = 5              # frames to "hold" a lock after the target disappears
