"""run.py — single entry point for the FLOW-UFO drone.

Launches the full drone program with follow_human as the active behavior:
  - Connects to drone WiFi
  - Loads camera + YOLO model
  - Calibrates gyro
  - Takes off and starts flying
  - Spins slowly to search for a person
  - Locks on and follows them at ~target_dist_ft (see config.json)
  - Q = gentle land   |   SPACE = emergency stop   |   Ctrl+C = gentle land oi

    python run.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def patch_config():
    """Ensure follow_human is the active behavior before launching."""
    config_path = os.path.join(HERE, "config.json")
    with open(config_path) as f:
        cfg = json.load(f)
    cfg["active_behavior"] = "follow_human"
    with open(config_path, "w") as f:
        json.dump(cfg, f, indent=2) 


if __name__ == "__main__":
    patch_config()

    # add the project folder to the path so all imports resolve
    sys.path.insert(0, HERE)

    # launch signals_control directly
    import signals_control
    signals_control.main()