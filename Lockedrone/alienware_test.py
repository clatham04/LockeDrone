"""Quick desktop test launcher.

Runs the exact same tracker as human_lock.py, but with the flexible .pt model and
a window — handy on a PC where you don't have an ncnn build for your CPU. A .pt
model accepts any INFER_SIZE without re-exporting.

Real settings live in config.py; this only overrides a couple for convenience.
"""
import config

config.MODEL_DIR = "yolo11n.pt"
config.SHOW_WINDOW = True

from human_lock import main   # imported after the overrides so they take effect

if __name__ == "__main__":
    main()
