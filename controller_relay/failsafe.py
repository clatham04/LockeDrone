r"""failsafe.py — the MANDATORY watchdog (its own component).

A control loop must NEVER leave the drone uncommanded. If the desktop's command link goes
silent for longer than `timeout_s`, the relay must stop forwarding the last (stale) command
and instead send a SAFE command every tick until the link returns.

  - action "hover"  -> centred sticks (128,128,128,128); the drone holds position.
  - action "land"   -> one-key auto-land (cooingdv F1 0x01); the drone descends and lands.

The command_relay's send loop calls Failsafe.command(...) on EVERY send, so the safe command
is applied continuously while the link is down — not just once.
"""
import time

CENTER = 128
F1_LAND = 0x01          # cooingdv GL: one-key takeoff/land toggle

HOVER_CMD = (CENTER, CENTER, CENTER, CENTER, 0, 0)        # (roll, pitch, throttle, yaw, f1, f2)
LAND_CMD = (CENTER, CENTER, CENTER, CENTER, F1_LAND, 0)


class Failsafe:
    def __init__(self, timeout_s=2.0, action="hover"):
        self.timeout_s = float(timeout_s)
        self.action = action
        self.safe = LAND_CMD if action == "land" else HOVER_CMD
        self._engaged = True        # start safe: no desktop command has arrived yet

    @property
    def engaged(self):
        return self._engaged

    def command(self, latest, latest_ts):
        """Return the command to actually send to the drone this tick:
        the desktop's latest command, or the SAFE command if the desktop has gone silent."""
        silent = (time.time() - latest_ts) > self.timeout_s
        if silent and not self._engaged:
            print(f"[FAILSAFE] desktop link SILENT > {self.timeout_s}s -> sending {self.action.upper()}")
            self._engaged = True
        elif not silent and self._engaged:
            print("[FAILSAFE] desktop link OK -> relaying commands")
            self._engaged = False
        return self.safe if self._engaged else latest
