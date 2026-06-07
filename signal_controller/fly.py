r"""fly.py — autonomous takeoff + hover (the base layer for future autonomy).

Sequence each run:
  1. gyro-calibrate   (drone must be FLAT and STILL on the ground)
  2. gentle takeoff   (capped low so it doesn't shoot into the ceiling)
  3. hover            (hold position as long as it can, until Ctrl+C -> land)

EXTENSIBLE BY DESIGN — this is the hook future code plugs into:
The hover loop asks a `controller(state)` for the four stick values every tick.
The default `hover_controller` just holds center. To add human-follow later, write a
controller that returns stick values from your vision data and run:

    import fly
    fly.run(controller=my_follow_controller)

...no changes to this file needed.

HONEST LIMIT: with no downward camera + floor marker, "hold" relies on the drone's
own optical flow, which drifts. The real lock-in-place hover is a position loop on a
downward camera — it slots in as a smarter `controller`. Until then: fly in OPEN
SPACE, props on, Ctrl+C lands it, and `wifi_control.py stop` is your kill switch.

    sudo python3 signal_controller/fly.py
"""
import time

import wifi_control as wc

CENTER = wc.CENTER                  # 128 = "hold" on every axis
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ

CALIBRATE_SECONDS = 1.5
TAKEOFF_SETTLE_THROTTLE = 90        # < 128 = descend, to cap the auto-takeoff climb
TAKEOFF_SETTLE_SECONDS = 1.5


def hover_controller(state):
    """Default behavior: hold center on every axis -> the drone hovers in place.

    state is None for now (no sensors). Future controllers receive sensor/vision
    data here and return adjusted sticks. Returns (roll, pitch, throttle, yaw).
    """
    return CENTER, CENTER, CENTER, CENTER


def _pulse(seconds, **kw):
    for _ in range(int(RATE_HZ * seconds)):
        wc.send(**kw)
        time.sleep(PERIOD)


def calibrate():
    print("[FLY] Put the drone FLAT and STILL on the ground. Calibrating gyro in 3s...")
    time.sleep(3)
    print("[FLY] calibrating...")
    _pulse(CALIBRATE_SECONDS, flags1=wc.F1_CALIBRATE)
    time.sleep(1.0)
    print("[FLY] calibration done.")


def takeoff():
    print("[FLY] taking off (gentle, low)...")
    _pulse(0.3, flags1=wc.F1_ONEKEY)                              # fire auto-takeoff
    _pulse(TAKEOFF_SETTLE_SECONDS, throttle=TAKEOFF_SETTLE_THROTTLE)  # cap the climb low
    print("[FLY] hovering — holding position. Ctrl+C to land.")


def land():
    print("\n[FLY] landing...")
    _pulse(0.3, flags1=wc.F1_ONEKEY)                              # one-key land
    _pulse(0.3, flags1=wc.F1_STOP)                               # then cut, to be safe


def run(controller=hover_controller):
    """Calibrate, take off, then hover using `controller` until Ctrl+C."""
    wc.setup()
    wc.start()                                                   # heartbeat thread
    try:
        calibrate()
        takeoff()
        secs = 0
        n = 0
        while True:
            roll, pitch, throttle, yaw = controller(None)
            wc.send(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
            n += 1
            if n % RATE_HZ == 0:
                secs += 1
                print(f"  ...hovering {secs}s")
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        land()
    finally:
        wc._running = False


if __name__ == "__main__":
    run()
