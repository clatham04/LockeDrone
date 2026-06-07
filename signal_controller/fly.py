r"""fly.py — autonomous takeoff + hover (the base layer for future autonomy).

Sequence: gyro-calibrate (flat & still) -> one-key takeoff (trim applied) -> hover
until Ctrl+C (land).

EXTENSIBLE: the hover loop asks `controller(state)` for the four stick values each
tick. Default holds center. Future human-follow supplies its own controller:
    import fly ; fly.run(controller=my_follow_controller)

========================= TUNING (edit these) =========================
LEFT/RIGHT DRIFT (it always slides one way) — a trim issue, not random drift:
    ROLL_TRIM   + leans RIGHT (cancels a LEFT slide).   try 8 -> 14 -> 18...
    PITCH_TRIM  + leans FORWARD (cancels a BACKWARD slide).
Also CALIBRATE on a genuinely flat, level floor — a consistent slide usually means
the drone learned a tilted "level".

TAKEOFF: this drone only arms via the one-key command, and its auto-takeoff climbs
fast (firmware — we can't slow it). Give it ceiling room. We layer the trim on during
liftoff so it fights the side-drift as early as the drone will accept it.
=======================================================================

HONEST LIMIT: without a downward camera + floor marker, "hold" leans on the drone's
own optical flow, which drifts. Fly in OPEN SPACE. Ctrl+C lands; in another terminal
`sudo python3 signal_controller/wifi_control.py stop` is the kill switch.

    sudo python3 signal_controller/fly.py
"""
import time

import wifi_control as wc

CENTER = wc.CENTER
LO, HI = 50, 200
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ

# --- tuning ---
ROLL_TRIM = 11         # + leans RIGHT to cancel a LEFT slide
PITCH_TRIM = 0         # + leans FORWARD to cancel a BACKWARD slide
CALIBRATE_SECONDS = 1.5
TAKEOFF_PULSE_SECONDS = 0.4    # how long to hold the one-key takeoff command
HOVER_DESCEND_THROTTLE = 110   # after takeoff, drop to a lower hover (LOWER = drops more)
HOVER_DESCEND_SECONDS = 1.0    # how long to descend before holding


def _clamp(v):
    return max(LO, min(HI, int(v)))


def send_trimmed(roll=CENTER, pitch=CENTER, throttle=CENTER, yaw=CENTER, flags1=0):
    """Send sticks with drift trim applied (so trim helps takeoff AND hover)."""
    wc.send(roll=_clamp(roll + ROLL_TRIM), pitch=_clamp(pitch + PITCH_TRIM),
            throttle=_clamp(throttle), yaw=_clamp(yaw), flags1=flags1)


def hover_controller(state):
    """Default: hold center on every axis. Returns (roll, pitch, throttle, yaw)."""
    return CENTER, CENTER, CENTER, CENTER


def _pulse(seconds, **kw):
    for _ in range(int(RATE_HZ * seconds)):
        wc.send(**kw)
        time.sleep(PERIOD)


def calibrate():
    print("[FLY] Put the drone FLAT and STILL on a LEVEL floor. Calibrating in 3s...")
    time.sleep(3)
    print("[FLY] calibrating gyro...")
    _pulse(CALIBRATE_SECONDS, flags1=wc.F1_CALIBRATE)
    time.sleep(1.0)
    print("[FLY] calibration done.")


def takeoff():
    print("[FLY] one-key takeoff (trim applied during liftoff)...")
    # one-key arms + auto-takeoffs; layer the trim on in case the drone honors it
    for _ in range(int(RATE_HZ * TAKEOFF_PULSE_SECONDS)):
        send_trimmed(flags1=wc.F1_ONEKEY)
        time.sleep(PERIOD)
    # the one-key takeoff altitude is a bit high — descend to a lower hover, then hold
    print("[FLY] descending to a lower hover...")
    for _ in range(int(RATE_HZ * HOVER_DESCEND_SECONDS)):
        send_trimmed(throttle=HOVER_DESCEND_THROTTLE)
        time.sleep(PERIOD)
    print("[FLY] hovering — holding with trim. Ctrl+C to land.")


def land():
    print("\n[FLY] landing...")
    _pulse(0.3, flags1=wc.F1_ONEKEY)        # one-key land
    _pulse(0.3, flags1=wc.F1_STOP)          # then cut, to be safe


def run(controller=hover_controller):
    """Calibrate, take off, then hover using `controller` until Ctrl+C."""
    wc.setup()
    wc.start()                               # heartbeat
    try:
        calibrate()
        takeoff()
        secs, n = 0, 0
        while True:
            roll, pitch, throttle, yaw = controller(None)
            send_trimmed(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)
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
