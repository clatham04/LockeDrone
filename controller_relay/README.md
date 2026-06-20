# controller_relay — Pi 4 drone ⇄ desktop relay

Low-latency bidirectional bridge on a Raspberry Pi 4. The Pi **does not process video** — it
forwards the drone's camera to the desktop and the desktop's commands to the drone, with a
**mandatory hover failsafe** if the desktop link drops.

```
        wlan0 (joins drone AP)                 eth0 (wired to desktop)
 DRONE ───── H.264 RTSP ─────►  PI 4 (relay)  ───── RTP/UDP video ─────►  DESKTOP (GPU)
 192.168.1.1                                                              detection
       ◄──── cooingdv UDP ─────         ◄──── 6-byte stick commands ──────
```

## Files — VIDEO path and COMMAND path are separate on purpose
| File | Path | What it does |
|---|---|---|
| `relay.py` | orchestrator | Loads config, starts everything, clean shutdown. **Run this.** |
| `video_relay.py` | **VIDEO** (drone→desktop) | GStreamer H.264 **passthrough**, no re-encode. The only file that changes if the camera isn't H.264. |
| `command_relay.py` | **COMMAND** (desktop→drone) | Listens for desktop sticks, builds the cooingdv packet, sends to drone at 25 Hz, runs the heartbeat. |
| `failsafe.py` | **SAFETY** | Watchdog: desktop silent > 2 s → hover (centered sticks). Consulted on every send. |
| `config.json` | config | Interfaces, IPs/ports, rates, failsafe timeout + action. |
| `desktop_stub.py` | desktop side | Minimal example of the contract: receive video, send sticks. Replace with your detector. |

## The two wire contracts (this is the whole interface)
- **Desktop → Pi (commands), UDP `:5700`:** 6 bytes `[roll, pitch, throttle, yaw, flags1, flags2]`, each 0–255, sticks centered at 128. `flags1`: `0x01` takeoff/land, `0x02` stop, `0x04` calibrate.
- **Pi → desktop (video), RTP/UDP `:5600`:** H.264 in RTP, payload type 96 (the drone's bitstream, untouched).
- **Pi → drone (internal), UDP `:7099`:** cooingdv "GL" 21-byte packet + `{0x01,0x01}` heartbeat. The desktop never sees this.

---

## One-time setup

### On the Pi
```bash
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base \
                    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```
- **wlan0:** join the drone's WiFi (`FLOW-UFO`), phone disconnected (single-client AP).
- **eth0:** give it a static IP on a subnet that is NOT `192.168.1.x` (avoid clashing with the drone), e.g. **`192.168.2.1/24`**.

### On the desktop
- **eth0:** static **`192.168.2.2/24`** (same subnet as the Pi's eth0).
- Edit `config.json` → `desktop.ip` = your desktop's eth0 IP.
- Edit `desktop_stub.py` → `PI_IP` = the Pi's eth0 IP.

> Confirm the camera codec once (expect `h264`):
> `ffprobe -v error -show_streams rtsp://192.168.1.1:7070/webcam | grep codec_name`

---

## Run it
**On the Pi** (root: `SO_BINDTODEVICE` pins the drone socket to wlan0):
```bash
sudo python3 relay.py
```
**On the desktop** — view the video:
```bash
gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=false
```
**On the desktop** — send commands (stub; swap in your detector's output):
```bash
python3 desktop_stub.py
```

## Failsafe behavior
- Startup: until the first desktop command arrives, the Pi sends **hover** — the drone can't run off.
- In flight: if the desktop goes silent > `failsafe.timeout_s` (2 s), the Pi sends **hover** every tick (`failsafe.action: "land"` to auto-land instead) and **keeps the heartbeat alive** — the drone is never uncommanded.
- Link returns → it resumes relaying the desktop's commands immediately.

## Where your real code goes
- **Desktop:** your detector receives video on `:5600`, runs on the GPU, and calls `send_command(...)` — i.e. the `signal_controller_PC` follow logic moves to the desktop and emits 6-byte sticks instead of talking to the drone directly.
- **Pi:** nothing to change — it's a dumb, safe pipe.
</content>
