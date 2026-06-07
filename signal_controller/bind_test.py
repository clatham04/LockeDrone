"""bind_test.py — PROPS OFF bind + idle test.

The first real end-to-end test: bind to the drone, then hold neutral sticks at zero
throttle so you can confirm the bind worked. Quit with Ctrl+C.

    python bind_test.py

SAFETY: remove the propellers before running. We keep throttle at zero, but a
successful bind means the drone is now taking commands from the Pi.

What to look for:
  - The drone's status LED stops blinking / goes solid  ->  BIND SUCCESS.
  - If it never binds: stop, set XN297_ADDRESS_REVERSED = True in drone_link.py,
    try again. Still nothing? See RF_LINK.md (Arduino + Multiprotocol fallback).
"""
import time

import bayang
import drone_link as link

# Set True (PROPS OFF!) to spin the motors gently and confirm throttle control.
SPIN_MOTORS = False
SPIN_THROTTLE = 0.12        # 0.0 - 1.0, only used if SPIN_MOTORS is True


def main():
    radio = link.connect()
    print("\n>>> Turn the drone OFF now.")
    print(">>> Binding starts in 3s — power the drone ON the moment you see 'Binding'.")
    print(">>> (Toy drones only accept a bind right after power-up.)\n")
    time.sleep(3.0)

    tx_id = link.TX_ID
    hops = link.bind(radio, tx_id, duration_s=12.0)   # long window so you can power-cycle during it

    thr = link.throttle(SPIN_THROTTLE) if SPIN_MOTORS else bayang.STICK_MIN
    neutral = bayang.STICK_CENTER
    period = bayang.PACKET_PERIOD_US / 1_000_000

    print(f"Holding sticks neutral, throttle={'SPIN' if SPIN_MOTORS else 'OFF'}. "
          f"Ctrl+C to stop.")
    hop = 0
    try:
        while True:
            hop = link.send_signal(radio, tx_id, hops, hop,
                                   aileron=neutral, elevator=neutral,
                                   throttle=thr, rudder=neutral)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping — cutting throttle.")
        # send a few zero-throttle packets so it doesn't keep the last command
        for _ in range(50):
            hop = link.send_signal(radio, tx_id, hops, hop,
                                   aileron=neutral, elevator=neutral,
                                   throttle=bayang.STICK_MIN, rudder=neutral)
            time.sleep(period)
        link.disconnect(radio)


if __name__ == "__main__":
    main()
