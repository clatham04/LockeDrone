# Signal Controller — WiFi drone control from a PC

Controls the **FLOW-UFO** drone (EIELE DIY kit / **KY UFO** app) from a **Windows PC**
over WiFi. `signals_control.py` connects to the drone, takes off, and runs a flight
**behavior** chosen in `config.json` — today that's `hover`; tomorrow you drop in
`follow_human` with no changes to the main script.

> The drone is a **cooingdv "GL"** variant. Control is **UDP port 7099**; the forward
> camera is **RTSP on TCP 7070**.

---

## Requirements
- Python 3.x with dependencies installed: `pip install -r requirements.txt`
- ffmpeg CLI added to your system PATH — download from https://ffmpeg.org/download.html
- Run your terminal **as Administrator** (needed for route commands)

---

## Run it
```
# drone ON, your phone disconnected from its WiFi (it's single-client), drone on a flat floor
# Run terminal as Administrator
python signals_control.py
```
It will: connect to the drone's WiFi (any SSID containing `flow`) → calibrate the
gyro → take off → hover → **Ctrl+C lands it gently.**

Kill switch (deliberate motor cut): `python wifi_control.py stop`

---

## Files
| File | Role |
|------|------|
| **`signals_control.py`** | **Main connector.** WiFi + calibrate + takeoff + control loop + trim + gentle land. Runs the active behavior. |
| **`config.json`** | Which behavior is active, the WiFi keyword, and the flight tuning (trim, hover height). |
| `behaviors/hover.py` | The hover behavior — holds center sticks. |
| `behaviors/follow_human.py` | **Vision follow.** Tracks a person and follows them (yaw to center, pitch for distance, throttle to stay head-level), never colliding. Hovers if no one is seen. |
| `wifi_control.py` | The cooingdv protocol (UDP 7099): build packets, heartbeat, send. Also a CLI: `calibrate` / `idle` / `takeoff` / `stop`. |
| `wifi_connect.py` | Find + connect to the drone's WiFi by name keyword. |
| `drone_camera.py` | Robust RTSP capture: decodes in an ffmpeg subprocess (survives the lossy stream, auto-restarts), pins the route, keeps the newest frame. |
| `camera_feed.py` | Quick one-shot grab of the forward camera (RTSP). |
| `detect_test.py` | Person-detection preview on the forward camera (no flying) — to check detection quality. |

---

## Tuning (edit `config.json`)
```json
"tuning": {
  "trim": { "roll": 9, "pitch": -8 },
  "hover_descend_throttle": 60,
  "hover_descend_seconds": 1.5
}
```
- **Drifts left/right** → adjust `trim.roll` (more positive = lean right).
- **Drifts fwd/back** → adjust `trim.pitch` (negative = lean back).
- **Hovers too high** → lower `hover_descend_throttle` or raise `hover_descend_seconds`.

---

## Follow-human (active behavior)
`config.json` ships with `active_behavior: "follow_human"`. Run `python signals_control.py`
and after takeoff the drone tracks the largest person it sees:
- **Yaw** turns to keep you centered. **Pitch** holds follow distance. **Throttle**
  keeps it level with your head. **No person / lost you → it spins slowly in place to
  search** until someone appears, then locks on (never flies off).
- **It won't run into you:** if you fill the frame / your whole body no longer fits, it
  backs off. Stand still and it holds position.

**Tuning** lives in `config.json → tuning.follow` (adjust outdoors, no code edits):
```json
"follow": {
  "target_dist_h": 0.60,
  "too_close_h": 0.78,
  "target_head_y": 0.28,
  "max_throttle": 24,
  "yaw_sign": 1, "pitch_sign": 1, "throttle_sign": 1
}
```
> ⚠️ **First flight: tie it to a string and start with low values.** If it turns/moves
> the wrong direction, flip that axis's `*_sign` to `-1`. Ctrl+C lands it gently.

---

## Add another behavior
1. Create `behaviors/follow_human.py` with:
   ```python
   def controller(state):
       return 128, 128, 128, 128
   def start(state, config): ...
   def stop(state, config): ...
   ```
2. Register + activate it in `config.json`:
   ```json
   "active_behavior": "follow_human",
   "behaviors": { "hover": "behaviors.hover", "follow_human": "behaviors.follow_human" }
   ```
That's it — `signals_control.py` loads it automatically.

---

## ⚠️ Notes
- **Run as Administrator** — the `route add` command used to reach the drone requires it.
- **Single-client WiFi:** disconnect your phone from the drone before running.
- **Subnet collision:** the home network and the drone's AP both use `192.168.1.x`.
  `wifi_control.py` pins its socket to the correct interface so packets reach the drone,
  not your router.
- **ffmpeg must be in PATH** — verify with `ffmpeg -version` in a terminal before running.
- **Fly in open space** — without the downward camera + floor marker, "hold" leans on the
  drone's own optical flow, which drifts. The trim cancels *constant* drift.