#!/usr/bin/env python3
r"""signals_control.py — main flight controller for the FLOW-UFO drone (Windows).

    python signals_control.py

Keys (active immediately after takeoff):
    Q     = gentle land
    SPACE = emergency motor cut
    Ctrl+C = gentle land
"""
import ctypes
import importlib
import json
import os
import threading
import time

import keyboard
import wifi_control as wc
import wifi_connect

HERE = os.path.dirname(os.path.abspath(__file__))
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ

_key_land = False
_key_stop = False
_keys_active = False


def _on_q():
    if _keys_active:
        globals().update(_key_land=True)


def _on_space():
    if _keys_active:
        globals().update(_key_stop=True)


def _reset_console():
    """Restore normal terminal input mode (echo, line buffering).

    cv2 windows + keyboard hooks can leave the Windows console in a broken
    state where typing afterward doesn't echo or work. This restores it.
    """
    try:
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-10)  # STD_INPUT_HANDLE
        ENABLE_ECHO_INPUT = 0x0004
        ENABLE_LINE_INPUT = 0x0002
        ENABLE_PROCESSED_INPUT = 0x0001
        mode = ctypes.c_ulong()
        kernel32.GetConsoleMode(handle, ctypes.byref(mode))
        new_mode = mode.value | ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT | ENABLE_PROCESSED_INPUT
        kernel32.SetConsoleMode(handle, new_mode)
    except Exception:
        pass


def load_config():
    with open(os.path.join(HERE, "config.json")) as f:
        return json.load(f)


def _clamp(v):
    return max(50, min(200, int(v)))


def make_send(trim):
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
    print("[CTRL] taking off...")
    _pulse(send, t.get("takeoff_pulse_seconds", 0.4), flags1=wc.F1_ONEKEY)
    climb_s = t.get("takeoff_climb_seconds", 0)
    if climb_s > 0:                                    # climb higher than the firmware's default
        print(f"[CTRL] climbing to flight height ({climb_s}s)...")
        _pulse(send, climb_s, throttle=t.get("takeoff_climb_throttle", 180))
    print("[CTRL] flying.")


def land(send):
    print("\n[CTRL] landing — descending slowly, do not interrupt...")
    try:
        _pulse(send, 3.0, throttle=55)
        _pulse(send, 1.0, throttle=50)
        _pulse(send, 0.5, flags1=wc.F1_STOP)
    except KeyboardInterrupt:
        print("[CTRL] force stopping motors...")
        _pulse(send, 0.5, flags1=wc.F1_STOP)
    print("[CTRL] landed.")


def main():
    global _keys_active

    cfg = load_config()
    t = cfg.get("tuning", {})

    ssid = wifi_connect.find_and_connect(cfg.get("wifi_name_contains", "flow"))
    if not ssid:
        print("[CTRL] no drone wifi — turn the drone on and disconnect your phone.")
        return
    print(f"[CTRL] on {ssid}")

    name = cfg["active_behavior"]
    behavior = importlib.import_module(cfg["behaviors"][name])
    print(f"[CTRL] behavior: {name}")

    send = make_send(t.get("trim", {}))
    wc.setup()
    wc.start()

    # register hotkeys immediately but gate them with _keys_active flag
    keyboard.add_hotkey("q", _on_q)
    keyboard.add_hotkey("space", _on_space)

    state = {}
    try:
        # load camera + model BEFORE takeoff so it's ready the moment we're airborne
        if hasattr(behavior, "start"):
            print("[CTRL] loading camera and model — please wait...")
            behavior.start(state, cfg)
            print("[CTRL] ready — starting calibration.")

        calibrate(send, t)
        takeoff(send, t)

        # activate keys only after drone is airborne
        _keys_active = True
        print("[CTRL] keys ready —  Q = gentle land   |   SPACE = emergency stop")

        secs = n = 0
        while True:
            if _key_stop:
                print("\n[CTRL] SPACE — emergency stop!")
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
        _keys_active = False
        keyboard.unhook_all()
        if hasattr(behavior, "stop"):
            try:
                behavior.stop(state, cfg)
            except Exception:
                pass
        wc._running = False
        print("[CTRL] done.")
        _reset_console()


if __name__ == "__main__":
    main()