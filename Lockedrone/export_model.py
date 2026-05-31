"""Re-export yolo11n.pt to an ncnn model baked at config.INFER_SIZE.

ncnn exports are FIXED size: the input resolution is compiled into the model, so
running at any other size corrupts memory and crashes
("malloc(): invalid size (unsorted)"). Run this whenever you change INFER_SIZE:

    python export_model.py

The output folder (yolo11n_ncnn_model/) is portable — you can export on a PC and
copy it to the Pi, or run this directly on the Pi.
"""
from pathlib import Path

from ultralytics import YOLO

import config

REPO_ROOT = Path(__file__).resolve().parent.parent


def main():
    weights = REPO_ROOT / "yolo11n.pt"
    if not weights.exists():
        raise FileNotFoundError(f"Base weights not found: {weights}")

    print(f"[EXPORT] Baking ncnn model at {config.INFER_SIZE}px from {weights.name}...")
    YOLO(str(weights)).export(format="ncnn", imgsz=config.INFER_SIZE)
    print(f"[EXPORT] Done -> {REPO_ROOT / 'yolo11n_ncnn_model'}")


if __name__ == "__main__":
    main()
