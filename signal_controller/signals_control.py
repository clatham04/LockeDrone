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

    sudo python3 signals_control.py
"""
import importlib
import json
import os
import time

import wifi_control as wc
import wifi_connect

HERE = os.path.dirname(os.path.abspath(__file__))
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ


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
    print("\n[CTRL] landing gently — wait for it to come down...")
    try:
        _pulse(send, 0.3, flags1=wc.F1_ONEKEY)     # trigger auto-land
        _pulse(send, 5.0)                          # hold link while it descends (no motor cut)
    except KeyboardInterrupt:
        pass                                       # don't abort the landing mid-air
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
    state = {}                                     # shared bag (future: camera/detections go here)
    try:
        # Start the behavior (load camera + model) on the GROUND, BEFORE flying. Doing this
        # after takeoff left the drone hovering with NO control commands for the seconds it
        # took to load — so it drifted off and crashed, and the tuning never even ran.
        if hasattr(behavior, "start"):
            try:
                behavior.start(state, cfg)
            except Exception as e:
                print(f"[CTRL] behavior '{name}' failed to start: {e} — NOT taking off.")
                return
        calibrate(send, t)
        takeoff(send, t)
        secs = n = 0
        while True:
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
        if hasattr(behavior, "stop"):
            try:
                behavior.stop(state, cfg)
            except Exception:
                pass
        wc._running = False


if __name__ == "__main__":
    main()
