r"""fly.py — manual keyboard flight for the FLOW-UFO drone (cooingdv GL over WiFi).

Reuses the working protocol from wifi_control.py. Sticks auto-recenter when you let
go, so releasing every key = the drone holds (altitude-hold hover).

  Left stick  (throttle + yaw):   W = up    S = down     A = yaw left   D = yaw right
  Right stick (pitch + roll):     I = fwd   K = back      J = roll left  L = roll right
  T      = take off / land (toggle)
  SPACE  = EMERGENCY STOP (cut motors NOW)
  Q      = land and quit

WARNING: props are on. Fly in an OPEN SPACE or strap it down. SPACE kills the motors.

    sudo python3 signal_controller/fly.py
"""
import curses
import time

import wifi_control as wc

CENTER = wc.CENTER          # 128
LO, HI = 50, 200            # stick bounds the app uses
STEP = 28                   # how far a held key pushes a stick from center
DECAY = 14                  # recenter speed per tick when the key is released
RATE_HZ = 25
PERIOD = 1.0 / RATE_HZ

# The one-key auto-takeoff climbs hard to a default height. Right after firing it,
# we push throttle below center for a moment to arrest that climb and settle low.
TAKEOFF_SETTLE_THROTTLE = 90      # < 128 = descend (lower = stops climb harder)
TAKEOFF_SETTLE_SECONDS = 1.5      # how long to hold the descend after takeoff


def _clamp(v):
    return max(LO, min(HI, int(v)))


def _toward(val, target):
    return _clamp(val + (target - val) * 0.5)


def _decay(val):
    if val > CENTER:
        return max(CENTER, val - DECAY)
    if val < CENTER:
        return min(CENTER, val + DECAY)
    return val


def _pulse(flags1, ticks=8):
    for _ in range(ticks):
        wc.send(flags1=flags1)
        time.sleep(0.03)


def main(stdscr):
    wc.setup()
    wc.start()                       # heartbeat thread
    stdscr.nodelay(True)
    curses.curs_set(0)
    stdscr.addstr(0, 0, "FLY  W/S thr  A/D yaw  I/K pitch  J/L roll   T takeoff/land   SPACE STOP   Q quit")

    roll = pitch = throttle = yaw = CENTER
    flying = False

    while True:
        # drain all keys pressed since the last tick (lowercased)
        chars = set()
        k = stdscr.getch()
        while k != -1:
            if 0 <= k < 0x110000:
                try:
                    chars.add(chr(k).lower())
                except ValueError:
                    pass
            k = stdscr.getch()

        # --- special actions ---
        if " " in chars:
            stdscr.addstr(4, 0, "*** EMERGENCY STOP ***                 ")
            stdscr.refresh()
            _pulse(wc.F1_STOP, 15)
            flying = False
            roll = pitch = throttle = yaw = CENTER
            continue
        if "q" in chars:
            stdscr.addstr(4, 0, "landing + quitting...                  ")
            stdscr.refresh()
            if flying:
                _pulse(wc.F1_ONEKEY)          # land
            _pulse(wc.F1_STOP)
            return
        if "t" in chars:
            flying = not flying
            if flying:
                stdscr.addstr(4, 0, "TAKEOFF (gentle — settling low)...     ")
                stdscr.refresh()
                _pulse(wc.F1_ONEKEY)                       # fire auto-takeoff
                # immediately fight the auto-climb so it stays low
                for _ in range(int(RATE_HZ * TAKEOFF_SETTLE_SECONDS)):
                    wc.send(throttle=TAKEOFF_SETTLE_THROTTLE)
                    time.sleep(PERIOD)
                throttle = CENTER                          # then hold this low altitude
            else:
                stdscr.addstr(4, 0, "LAND ...                               ")
                stdscr.refresh()
                _pulse(wc.F1_ONEKEY)
                throttle = CENTER

        # --- movement: held key pushes the stick, released axis recenters ---
        def axis(neg, pos, val):
            if pos in chars:
                return _toward(val, CENTER + STEP)
            if neg in chars:
                return _toward(val, CENTER - STEP)
            return _decay(val)

        throttle = axis("s", "w", throttle)
        yaw      = axis("a", "d", yaw)
        pitch    = axis("k", "i", pitch)
        roll     = axis("j", "l", roll)

        wc.send(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)

        stdscr.addstr(2, 0, f"R:{roll:3d}  P:{pitch:3d}  T:{throttle:3d}  Y:{yaw:3d}   flying:{flying}    ")
        stdscr.refresh()
        time.sleep(PERIOD)


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    finally:
        wc._running = False          # stop the heartbeat thread on exit
