r"""fly.py — autonomous takeoff + hover (the base layer for future autonomy).

Sequence: gyro-calibrate (flat & still) -> gentle takeoff -> hover until Ctrl+C (land).

EXTENSIBLE: the hover loop asks `controller(state)` for the four stick values each
tick. Default holds center. Future human-follow code supplies its own controller:
    import fly ; fly.run(controller=my_follow_controller)

============================ TUNING (edit these) ============================
LEFT/RIGHT DRIFT: if it always slides one way, that's a trim issue, not random drift.
  ROLL_TRIM  +leans RIGHT (cancels a LEFT slide).  PITCH_TRIM +leans FORWARD.
TAKEOFF too fast: the drone's one-key auto-takeoff ignores throttle while climbing,
  so we ramp throttle up ourselves (MANUAL_TAKEOFF). If the motors WON'T spin that
  way (some firmwares only arm via one-key), set MANUAL_TAKEOFF=False — but then the
  climb is the drone's aggressive auto one, so use a tall space.
=============================================================================

HONEST LIMIT: without a downward camera + floor marker, "hold" leans on the drone's
own optical flow, which drifts. Fly in OPEN SPACE. Ctrl+C lands; `wifi_control.py
stop` is the kill switch.

    sudo python3 signal_controller/fly.py
"""
import time

import wifi_control as wc

CENTER = wc.CENTER
LO, HI = 50, 200
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ

# --- tuning ---
ROLL_TRIM = 8          # + = lean RIGHT to cancel a LEFT slide. start ~8, raise if still left.
PITCH_TRIM = 0         # + = lean FORWARD to cancel a BACKWARD slide.

MANUAL_TAKEOFF = True              # ramp throttle ourselves (gentle). False = one-key (aggressive).
MANUAL_TAKEOFF_THROTTLE = 165      # peak throttle during the gentle ramp (raise if it won't lift)
MANUAL_TAKEOFF_SECONDS = 2.5       # ramp duration (longer = slower, gentler climb)

CALIBRATE_SECONDS = 1.5
ONEKEY_SETTLE_THROTTLE = 70        # used only if MANUAL_TAKEOFF=False
ONEKEY_SETTLE_SECONDS = 1.5


def _clamp(v):
    return max(LO, min(HI, int(v)))


def send_trimmed(roll=CENTER, pitch=CENTER, throttle=CENTER, yaw=CENTER):
    """Send sticks with the drift trim applied (so trim helps takeoff AND hover)."""
    wc.send(roll=_clamp(roll + ROLL_TRIM), pitch=_clamp(pitch + PITCH_TRIM),
            throttle=_clamp(throttle), yaw=_clamp(yaw))


def hover_controller(state):
    """Default: hold center on every axis. Returns (roll, pitch, throttle, yaw)."""
    return CENTER, CENTER, CENTER, CENTER


def _pulse(seconds, **kw):
    for _ in range(int(RATE_HZ * seconds)):
        wc.send(**kw)
        time.sleep(PERIOD)


def calibrate():
    print("[FLY] Put the drone FLAT and STILL on a LEVEL surface. Calibrating in 3s...")
    time.sleep(3)
    print("[FLY] calibrating gyro...")
    _pulse(CALIBRATE_SECONDS, flags1=wc.F1_CALIBRATE)
    time.sleep(1.0)
    print("[FLY] calibration done.")


def takeoff():
    if MANUAL_TAKEOFF:
        print("[FLY] manual takeoff — ramping throttle gently (with trim)...")
        ticks = int(RATE_HZ * MANUAL_TAKEOFF_SECONDS)
        for i in range(ticks):
            thr = CENTER + (MANUAL_TAKEOFF_THROTTLE - CENTER) * (i + 1) / ticks
            send_trimmed(throttle=thr)
            time.sleep(PERIOD)
        print("[FLY] airborne (if motors spun) — holding.")
        print("      If it did NOT lift, set MANUAL_TAKEOFF=False (or raise MANUAL_TAKEOFF_THROTTLE).")
    else:
        print("[FLY] one-key takeoff (aggressive — needs a tall space)...")
        _pulse(0.3, flags1=wc.F1_ONEKEY)
        _pulse(ONEKEY_SETTLE_SECONDS, throttle=ONEKEY_SETTLE_THROTTLE)
    print("[FLY] hovering — holding position. Ctrl+C to land.")


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
