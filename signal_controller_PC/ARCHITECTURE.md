# Signal Controller — Architecture

Design goal (now real): **`signals_control.py` is a dumb, reliable connector; the
interesting, swappable flight logic lives in small `behaviors/` plug-ins, chosen in
`config.json`.**

---

## The stack

```
   config.json ──► signals_control.py ──► behaviors/<active>.py
   (what to run)    (the CONNECTOR)        (decides the 4 sticks)
                          │
                          ▼
                    wifi_control.py  ── UDP 7099 ──►  DRONE
                    (cooingdv protocol)               192.168.1.1
                          ▲
                    wifi_connect.py
                    (join the "flow" WiFi)
```

- **`wifi_connect.py`** — scans for the drone's AP (SSID contains `flow`) and connects.
- **`wifi_control.py`** — the reverse-engineered **cooingdv GL** protocol: builds the
  21-byte control packet (`03 66 14 RR PP TT YY F1 F2 …00… CK 99`), runs the
  `{0x01,0x01}` heartbeat, and sends over UDP 7099 (socket pinned to `wlan0`).
- **`signals_control.py`** — the connector. Owns wifi, calibration, takeoff, the loop,
  drift trim, and a gentle landing.
- **`behaviors/*.py`** — pure decision logic. One function: `controller(state)`.

---

## The control loop (`signals_control.py`)

```
load config.json
connect to drone wifi (wifi_connect)
import the active behavior (importlib, from config["behaviors"][active])
wc.setup(); wc.start()            # open socket + start heartbeat
calibrate()                       # gyro cal (drone flat & still)
takeoff()                         # one-key takeoff + settle to hover height
behavior.start(state, config)     # optional
loop @ 25 Hz:
    roll,pitch,throttle,yaw = behavior.controller(state)
    send(...)                     # trim applied here, then UDP to the drone
on Ctrl+C: land()                 # auto-land, hold link while it descends (no motor cut)
behavior.stop(state, config)      # optional
```

`send()` applies the **drift trim** from config to every packet, so trim helps both
takeoff and every behavior uniformly. `state` is a shared dict — empty today, the
place where camera frames / detections will live for vision behaviors.

---

## The behavior contract

A behavior is any module under `behaviors/` that defines:

```python
def controller(state):
    """Called every tick. Return (roll, pitch, throttle, yaw). 128 = center/hold."""
    return 128, 128, 128, 128

def start(state, config):   # OPTIONAL — once, before the loop
    ...

def stop(state, config):    # OPTIONAL — once, on exit
    ...
```

That's the entire abstraction. If you can write `controller(state) -> 4 sticks`, you
can add a drone behavior without touching the connector.

- `hover` returns center → the drone holds via its own optical flow.
- `follow_human` (built) reads the forward camera via `drone_camera.DroneCamera`, runs
  YOLO in its own thread, and returns yaw (center the person) / pitch (hold distance,
  never collide) / throttle (stay head-level). No person → it returns center (hover).
  Detection runs off-loop so the 25 Hz control stays smooth and a video glitch → hover.
- A future `position_hold` will read a **downward camera + floor marker** and return
  roll/pitch to cancel drift — the true lock-in-place hover.

---

## config.json

```json
{
  "wifi_name_contains": "flow",        // which AP to join
  "active_behavior": "hover",          // which behavior to run
  "behaviors": { "hover": "behaviors.hover" },   // name -> import path
  "tuning": {
    "trim": { "roll": 9, "pitch": -8 },          // cancel constant drift
    "calibrate_seconds": 1.5,
    "takeoff_pulse_seconds": 0.4,
    "hover_descend_throttle": 60,                // lower = lower hover
    "hover_descend_seconds": 1.5
  }
}
```

---

## Why this shape
- **Behaviors are swappable** without touching wifi/takeoff/landing/safety.
- **Config-driven** — change what flies (and how it's tuned) without editing code.
- **The hard, dangerous parts live in one audited place** (the connector). New behaviors
  can't break takeoff or landing; they only choose sticks.

## Known limits / next
- "Hold" currently leans on the drone's drifting optical flow. The fix is a
  **downward-camera position-hold behavior** (camera + ArUco floor marker) — it slots
  in as just another `controller`.
- Subnet collision (home + drone both `192.168.1.x`) is handled by pinning the control
  socket to `wlan0` in `wifi_control.py`.


The drone this time just stayed in place and didnt move at all when i was in front of it. i even got closer and moved around and it didn't lock on to me at all.

