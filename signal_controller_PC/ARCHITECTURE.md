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


[WIFI] already connected to 'FLOW-UFO-44a562'
[CTRL] on FLOW-UFO-44a562
[CTRL] behavior: follow_human
[WIFI] socket bound to 192.168.1.100
[WIFI] heartbeat + control -> 192.168.1.1:7099
[CTRL] loading camera and model — please wait...
[FOLLOW] starting camera + detector...
[CAM] stream size: 640x352
[FOLLOW] model ready @ 256px — searching for a person...
[FOLLOW] spinning to search. Will lock on when someone comes into view.
[CTRL] ready — starting calibration.
[CTRL] Put the drone FLAT and STILL on a level floor. Calibrating in 3s...
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
[CTRL] calibrating gyro...
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
[CTRL] calibrated.
[CTRL] taking off...
[FOLLOW] no person detected — searching...
[CTRL] flying.
[CTRL] keys ready —  Q = gentle land   |   SPACE = emergency stop
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 1s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 2s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 3s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 4s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 5s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 6s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 7s
[FOLLOW] no person detected — searching...
  ...follow_human 8s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 9s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 10s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 11s
[FOLLOW] no person detected — searching...
[FOLLOW] person — cx:0.89 h:0.81 top:0.01
[FOLLOW] pitch=128 (HOLD)  dist=10.9ft  target=11.0ft
  ...follow_human 12s
[FOLLOW] person — cx:0.50 h:0.86 top:0.00
[FOLLOW] person — cx:0.52 h:0.81 top:0.01
[FOLLOW] pitch=128 (HOLD)  dist=10.6ft  target=11.0ft
  ...follow_human 13s
[FOLLOW] person — cx:0.54 h:0.75 top:0.00
[FOLLOW] person — cx:0.56 h:0.88 top:0.11
[FOLLOW] pitch=128 (HOLD)  dist=11.5ft  target=11.0ft
  ...follow_human 14s
[FOLLOW] person — cx:0.57 h:0.84 top:0.09
[FOLLOW] person — cx:0.58 h:0.85 top:0.05
[FOLLOW] pitch=143 (FORWARD)  dist=11.6ft  target=11.0ft
  ...follow_human 15s
[FOLLOW] person — cx:0.62 h:0.69 top:0.31
[FOLLOW] pitch=128 (HOLD)  dist=11.2ft  target=11.0ft
[FOLLOW] person — cx:0.63 h:0.72 top:0.27
  ...follow_human 16s
[FOLLOW] person — cx:0.63 h:0.68 top:0.31
[FOLLOW] pitch=88 (BACKWARD)  dist=9.5ft  target=11.0ft
[FOLLOW] person — cx:0.61 h:0.78 top:0.19
  ...follow_human 17s
[FOLLOW] person — cx:0.63 h:0.57 top:0.43
[FOLLOW] pitch=145 (FORWARD)  dist=11.7ft  target=11.0ft
[FOLLOW] person — cx:0.61 h:0.77 top:0.05
  ...follow_human 18s
[FOLLOW] person — cx:0.64 h:0.59 top:0.00
[FOLLOW] pitch=78 (BACKWARD)  dist=9.0ft  target=11.0ft
[FOLLOW] person — cx:0.68 h:0.71 top:0.11
  ...follow_human 19s
[FOLLOW] person — cx:0.71 h:0.49 top:0.50
[FOLLOW] pitch=178 (FORWARD)  dist=13.1ft  target=11.0ft
  ...follow_human 20s
[FOLLOW] person — cx:0.63 h:0.56 top:0.42
[FOLLOW] person — cx:0.64 h:0.68 top:0.11
[FOLLOW] pitch=172 (FORWARD)  dist=12.7ft  target=11.0ft
  ...follow_human 21s
[FOLLOW] person — cx:0.66 h:0.70 top:0.28
[FOLLOW] person — cx:0.75 h:0.22 top:0.78
[FOLLOW] pitch=78 (BACKWARD)  dist=8.2ft  target=11.0ft
  ...follow_human 22s
[FOLLOW] person — cx:0.75 h:0.42 top:0.58
[FOLLOW] person — cx:0.70 h:0.64 top:0.36
[FOLLOW] pitch=128 (HOLD)  dist=10.6ft  target=11.0ft
  ...follow_human 23s
[FOLLOW] person — cx:0.82 h:0.31 top:0.69
[FOLLOW] person — cx:0.71 h:0.50 top:0.50
[FOLLOW] pitch=78 (BACKWARD)  dist=8.0ft  target=11.0ft
  ...follow_human 24s
[FOLLOW] person — cx:0.81 h:0.67 top:0.31
[FOLLOW] person — cx:0.61 h:0.61 top:0.27
[FOLLOW] pitch=178 (FORWARD)  dist=14.2ft  target=11.0ft
  ...follow_human 25s
[FOLLOW] person — cx:0.64 h:0.61 top:0.28
[FOLLOW] person — cx:0.67 h:0.42 top:0.57
[FOLLOW] pitch=128 (HOLD)  dist=10.7ft  target=11.0ft
  ...follow_human 26s
[FOLLOW] person — cx:0.66 h:0.57 top:0.43
[FOLLOW] person — cx:0.69 h:0.46 top:0.53
[FOLLOW] pitch=147 (FORWARD)  dist=11.8ft  target=11.0ft
  ...follow_human 27s
[FOLLOW] person — cx:0.69 h:0.57 top:0.36
[FOLLOW] person — cx:0.68 h:0.54 top:0.42
[FOLLOW] pitch=177 (FORWARD)  dist=12.9ft  target=11.0ft
  ...follow_human 28s
[FOLLOW] person — cx:0.66 h:0.54 top:0.22
[FOLLOW] person — cx:0.68 h:0.57 top:0.35
[FOLLOW] pitch=178 (FORWARD)  dist=14.6ft  target=11.0ft
  ...follow_human 29s
[FOLLOW] person — cx:0.68 h:0.55 top:0.38
[FOLLOW] pitch=178 (FORWARD)  dist=15.1ft  target=11.0ft
[FOLLOW] person — cx:0.70 h:0.51 top:0.48
  ...follow_human 30s
[FOLLOW] person — cx:0.70 h:0.57 top:0.42
[FOLLOW] pitch=178 (FORWARD)  dist=14.8ft  target=11.0ft
[FOLLOW] person — cx:0.68 h:0.58 top:0.38
  ...follow_human 31s
[FOLLOW] person — cx:0.69 h:0.62 top:0.37
[FOLLOW] pitch=175 (FORWARD)  dist=12.9ft  target=11.0ft
  ...follow_human 32s
[FOLLOW] person — cx:0.62 h:0.43 top:0.56
[FOLLOW] person — cx:0.59 h:0.40 top:0.60
[FOLLOW] pitch=162 (FORWARD)  dist=12.4ft  target=11.0ft
  ...follow_human 33s
[FOLLOW] person — cx:0.51 h:0.30 top:0.70
[FOLLOW] no person detected — searching...
  ...follow_human 34s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 35s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 36s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 37s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 38s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 39s
[FOLLOW] no person detected — searching...
  ...follow_human 40s
[FOLLOW] no person detected — searching...
[FOLLOW] no person detected — searching...
  ...follow_human 41s
[FOLLOW] person — cx:0.95 h:0.56 top:0.07
[FOLLOW] person — cx:0.47 h:0.54 top:0.03
[FOLLOW] pitch=178 (FORWARD)  dist=14.1ft  target=11.0ft
  ...follow_human 42s
[FOLLOW] person — cx:0.18 h:0.40 top:0.00
[FOLLOW] person — cx:0.39 h:0.49 top:0.20
[FOLLOW] pitch=178 (FORWARD)  dist=14.2ft  target=11.0ft
  ...follow_human 43s
[FOLLOW] person — cx:0.61 h:0.38 top:0.02
[FOLLOW] person — cx:0.57 h:0.34 top:0.00
[FOLLOW] pitch=178 (FORWARD)  dist=19.3ft  target=11.0ft
  ...follow_human 44s
[FOLLOW] person — cx:0.53 h:0.39 top:0.01
[FOLLOW] pitch=178 (FORWARD)  dist=17.5ft  target=11.0ft
[FOLLOW] person — cx:0.54 h:0.38 top:0.07
  ...follow_human 45s
[FOLLOW] person — cx:0.54 h:0.38 top:0.11
[FOLLOW] pitch=178 (FORWARD)  dist=21.3ft  target=11.0ft
[FOLLOW] person — cx:0.54 h:0.39 top:0.17

[CTRL] landing — descending slowly, do not interrupt...
[FOLLOW] person — cx:0.56 h:0.41 top:0.27
[FOLLOW] person — cx:0.58 h:0.38 top:0.62
[FOLLOW] person — cx:0.60 h:0.39 top:0.57
[FOLLOW] person — cx:0.63 h:0.40 top:0.43
[FOLLOW] person — cx:0.67 h:0.42 top:0.20
[FOLLOW] person — cx:0.71 h:0.37 top:0.09
[FOLLOW] person — cx:0.76 h:0.38 top:0.01
[FOLLOW] person — cx:0.79 h:0.31 top:0.00
[CTRL] landed.
[FOLLOW] stopped.
[CTRL] done.

the drone was locking on to me and tracking me, but it is a little windy and started blowing away. edit code to have some wind resistance also since the dron is pretty high up, adijust the q (slow landing) to land comepletly. its sutting off in the air and falling.
