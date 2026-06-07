# Signal Controller — WiFi drone control from the Pi

Controls the **FLOW-UFO** drone (EIELE DIY kit / **KY UFO** app) from a Raspberry Pi
over WiFi. `signals_control.py` connects to the drone, takes off, and runs a flight
**behavior** chosen in `config.json` — today that's `hover`; tomorrow you drop in
`follow_human` with no changes to the main script.

> The drone is a **cooingdv "GL"** variant. Control is **UDP port 7099**; the forward
> camera is **RTSP on TCP 7070**. (The RF/nRF24 path was abandoned — WiFi is the way.)

---

## Run it
```bash
# drone ON, your phone disconnected from its WiFi (it's single-client), drone on a flat floor
sudo python3 signals_control.py
```
It will: find + join the drone's WiFi (any SSID containing `flow`) → calibrate the
gyro → take off → hover → **Ctrl+C lands it gently.**

Kill switch (deliberate motor cut): `sudo python3 wifi_control.py stop`

---

## Files
| File | Role |
|------|------|
| **`signals_control.py`** | **Main connector.** WiFi + calibrate + takeoff + control loop + trim + gentle land. Runs the active behavior. |
| **`config.json`** | Which behavior is active, the WiFi keyword, and the flight tuning (trim, hover height). |
| `behaviors/hover.py` | The hover behavior — holds center sticks. |
| `wifi_control.py` | The cooingdv protocol (UDP 7099): build packets, heartbeat, send. Also a CLI: `calibrate` / `idle` / `takeoff` / `stop`. |
| `wifi_connect.py` | Find + connect to the drone's WiFi by name keyword. |
| `camera_feed.py` | Pull the drone's forward camera (RTSP). |

---

## Tuning (edit `config.json`)
```json
"tuning": {
  "trim": { "roll": 9, "pitch": -8 },   // cancel constant drift: +roll=right, -pitch=back
  "hover_descend_throttle": 60,          // lower = lower hover (50 = strongest descend)
  "hover_descend_seconds": 1.5
}
```
- **Drifts left/right** → adjust `trim.roll` (more positive = lean right).
- **Drifts fwd/back** → adjust `trim.pitch` (negative = lean back).
- **Hovers too high** → lower `hover_descend_throttle` or raise `hover_descend_seconds`.

---

## Add a behavior (e.g. human-follow)
1. Create `behaviors/follow_human.py` with:
   ```python
   def controller(state):
       # read state (camera/detections), return (roll, pitch, throttle, yaw), center 128
       return 128, 128, 128, 128
   def start(state, config): ...   # optional
   def stop(state, config): ...    # optional
   ```
2. Register + activate it in `config.json`:
   ```json
   "active_behavior": "follow_human",
   "behaviors": { "hover": "behaviors.hover", "follow_human": "behaviors.follow_human" }
   ```
That's it — `signals_control.py` loads it automatically. See [ARCHITECTURE.md](ARCHITECTURE.md).

---

## ⚠️ Notes
- **Single-client WiFi:** disconnect your phone from the drone before running.
- **Subnet collision:** the home network and the drone's AP both use `192.168.1.x`.
  `wifi_control.py` pins its socket to `wlan0` so packets reach the drone, not your router.
- **Fly in open space** — without the downward camera + floor marker, "hold" leans on the
  drone's own optical flow, which drifts. The trim cancels *constant* drift; the downward
  camera (a future behavior) is what makes it lock in place.
