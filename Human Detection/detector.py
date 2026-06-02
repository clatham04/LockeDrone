"""Person detection with a YOLO model (ncnn folder on the Pi, or a .pt for testing).

Two plain functions: load the model once, then call detect_person each frame.
"""
from pathlib import Path

import numpy as np
import yaml
from ultralytics import YOLO

# Repo root = one level up from this file (the folder that holds yolo11n_ncnn_model/).
REPO_ROOT = Path(__file__).resolve().parent.parent


def load_model(model_dir):
    """Load a YOLO model and report the size it was exported at.

    Returns (model, native_size). `native_size` is the input resolution baked into
    an ncnn export (read from its metadata.yaml), or None for a .pt file. The path
    is resolved against the repo root, so it works no matter which directory you
    launch from — and we never silently fall back to downloading weights.
    """
    path = Path(model_dir)
    if not path.is_absolute():
        path = REPO_ROOT / model_dir
    if not path.exists():
        raise FileNotFoundError(f"Model not found at: {path}")

    native_size = None
    meta = path / "metadata.yaml"
    if path.is_dir() and meta.exists():
        info = yaml.safe_load(meta.read_text())
        size = info.get("imgsz")
        native_size = size[0] if isinstance(size, (list, tuple)) else size

    return YOLO(str(path), task="detect"), native_size


def detect_person(model, frame, conf, imgsz, device, person_class=0):
    """Run inference and return the most confident person box.

    Returns (x1, y1, x2, y2, confidence) or None if no person is found.
    """
    results = model(
        frame,
        conf=conf,
        classes=[person_class],
        imgsz=imgsz,
        device=device,
        verbose=False,
    )
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return None

    confs = boxes.conf.cpu().numpy()
    best = int(np.argmax(confs))
    x1, y1, x2, y2 = (int(v) for v in boxes.xyxy[best].cpu().numpy())
    return x1, y1, x2, y2, float(confs[best])
