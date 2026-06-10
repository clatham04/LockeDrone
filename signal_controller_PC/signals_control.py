#!/usr/bin/env python3
r"""signals_control.py — main flight controller (the connector).

Connects to the drone's WiFi, then flies the BEHAVIOR chosen in config.json. This
file owns the boring/critical parts — wifi, calibration, takeoff, the control loop,
drift trim, and a gentle landing. Behaviors only decide the four sticks each tick.

A behavior is a module under behaviors/ exposing:
    controller(state) -> (roll, pitch, throttle, yaw)   # required, called each tick
    start(state, config)                                # optional
    stop(state, config)                                 # optional

To add one (e.g. follow_human): drop behaviors/follow_human.py, register it in
config.json's "behaviors", and set "active_behavior". No changes to this file.

    python signals_control.py
"""
import importlib
import json
import os
import threading
import time

import wifi_control as wc
import wifi_connect

HERE = os.path.dirname(os.path.abspath(__file__))
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ

# keyboard state — set by the listener thread
_key_land = False
_key_stop = False


def _keyboard_listener():
    """Background thread: watches for Q (land) and Space (emergency stop)."""
    global _key_land, _key_stop
    import keyboard
    print("[CTRL] keys ready —  Q = gentle land   |   SPACE = emergency stop")
    keyboard.add_hotkey("q", lambda: globals().update(_key_land=True))
    keyboard.add_hotkey("space", lambda: globals().update(_key_stop=True))
    keyboard.wait()  # block this thread, keeping hooks alive


def load_config():
    with open(os.path.join(HERE, "config.json")) as f:
        return json.load(f)


def _clamp(v):
    return max(50, min(200, int(v)))


def make_send(trim):
    """Return send() that applies the configured roll/pitch trim to every packet."""
    roll_trim, pitch_trim = trim.get("roll", 0), trim.get("pitch", 0)

    def send(roll=128, pitch=128, throttle=128, yaw=128, flags1=0):
        wc.send(roll=_clamp(roll + roll_trim), pitch=_clamp(pitch + pitch_trim),
                throttle=_clamp(throttle), yaw=_clamp(yaw), flags1=flags1)
    return send


def _pulse(send, seconds, **kw):
    for _ in range(int(RATE_HZ * seconds)):
        send(**kw)
        time.sleep(PERIOD)


def calibrate(send, t):
    print("[CTRL] Put the drone FLAT and STILL on a level floor. Calibrating in 3s...")
    time.sleep(3)
    print("[CTRL] calibrating gyro...")
    _pulse(send, t.get("calibrate_seconds", 1.5), flags1=wc.F1_CALIBRATE)
    time.sleep(1.0)
    print("[CTRL] calibrated.")


def takeoff(send, t):
    print("[CTRL] one-key takeoff (trim applied during liftoff)...")
    _pulse(send, t.get("takeoff_pulse_seconds", 0.4), flags1=wc.F1_ONEKEY)
    print("[CTRL] settling to hover height...")
    _pulse(send, t.get("hover_descend_seconds", 1.5),
           throttle=t.get("hover_descend_throttle", 90))
    print("[CTRL] flying.")


def land(send):
    """Controlled descent: gradually lower throttle, then cut motors."""
    print("\n[CTRL] landing — descending slowly, do not interrupt...")
    try:
        # step 1: descend slowly over ~3 seconds (throttle below center = descend)
        _pulse(send, 3.0, throttle=55)
        # step 2: cut throttle all the way for final drop to ground
        _pulse(send, 1.0, throttle=50)
        # step 3: motor stop
        _pulse(send, 0.5, flags1=wc.F1_STOP)
    except KeyboardInterrupt:
        # if Ctrl+C again during landing, force stop immediately
        print("[CTRL] force stopping motors...")
        _pulse(send, 0.5, flags1=wc.F1_STOP)
    print("[CTRL] landed.")


def main():
    cfg = load_config()
    t = cfg.get("tuning", {})

    # 1. connect to the drone's wifi (SSID contains e.g. "flow")
    ssid = wifi_connect.find_and_connect(cfg.get("wifi_name_contains", "flow"))
    if not ssid:
        print("[CTRL] no drone wifi — turn the drone on and disconnect your phone.")
        return
    print(f"[CTRL] on {ssid}")

    # 2. load the active behavior from config
    name = cfg["active_behavior"]
    behavior = importlib.import_module(cfg["behaviors"][name])
    print(f"[CTRL] behavior: {name}")

    # 3. fly
    send = make_send(t.get("trim", {}))
    wc.setup()
    wc.start()                                     # heartbeat thread
    threading.Thread(target=_keyboard_listener, daemon=True).start()
    state = {}                                     # shared bag (future: camera/detections go here)
    try:
        calibrate(send, t)
        takeoff(send, t)
        if hasattr(behavior, "start"):
            behavior.start(state, cfg)
        secs = n = 0
        while True:
            if _key_stop:
                print("\n[CTRL] SPACE pressed — emergency stop!")
                _pulse(send, 0.5, flags1=wc.F1_STOP)
                break
            if _key_land:
                land(send)
                break
            roll, pitch, throttle, yaw = behavior.controller(state)
            send(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
            n += 1
            if n % RATE_HZ == 0:
                secs += 1
                print(f"  ...{name} {secs}s")
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        land(send)
    finally:
        # stop behavior and heartbeat cleanly — no more printing after this
        if hasattr(behavior, "stop"):
            try:
                behavior.stop(state, cfg)
            except Exception:
                pass
        wc._running = False
        print("[CTRL] done.")


if __name__ == "__main__":
    main()