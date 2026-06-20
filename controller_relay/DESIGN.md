# controller_relay — Pi 4 bidirectional relay (drone ⇄ desktop)

The Pi 4 is a **dumb, low-latency bridge**. It does **not** process images. It forwards the
drone's video to the desktop, forwards the desktop's commands to the drone, and runs a
**failsafe watchdog** so a dropped desktop link never leaves the drone uncommanded.

```
            wlan0 (drone AP, client)            eth0 (desktop, LAN)
  DRONE  ───────── video ───────►  PI 4  ───────── video ───────►  DESKTOP (GPU)
  192.168.1.1                     relay                            detection
         ◄──────── commands ──────       ◄──────── commands ───────
```

- `wlan0` → drone link (the Pi joins the drone's WiFi AP)
- `eth0`  → desktop link (wired LAN to the brain)
- One WiFi radio only: wlan0 is the drone, eth0 is the desktop. Non-negotiable.

---

## Two clearly-separated paths + a watchdog

| Component | File (planned) | Job |
|---|---|---|
| **VIDEO path** | `video_relay.py` | Pull drone video on wlan0, forward to desktop on eth0. **Passthrough, NO re-encode** if the camera is already H.264. |
| **COMMAND path** | `command_relay.py` | Receive commands from desktop on eth0, send to drone on wlan0. Owns the drone heartbeat. |
| **FAILSAFE** | `failsafe.py` | Watchdog: no desktop command for `failsafe_timeout_s` (default 2.0) → send the drone a SAFE command (hover/land). MANDATORY. |
| Orchestrator | `relay.py` | Starts all three, wires config, clean shutdown. |
| Config | `config.json` | Interfaces, drone IP/port, desktop ports, failsafe timeout + action. |
| Desktop contract | `desktop_stub.py` | Minimal desktop example: receive video + send commands, so the Pi⇄desktop contract is explicit. |

---

## What I already know from our drone work (to CONFIRM, not guess)

These come from the working `signal_controller_PC` code, not assumptions:

- **Camera:** the drone streams **H.264 over RTSP** at `rtsp://192.168.1.1:7070/webcam`
  (that's exactly what we decode today with ffmpeg/`-f rawvideo`). → **Ideal case: relay the
  H.264 bitstream straight through with no re-encode on the Pi.**
- **Drone control:** the **cooingdv "GL"** protocol — **UDP port 7099**, 21-byte packet
  `03 66 14 RR PP TT YY F1 F2 00×10 CK 99` (sticks centred at 128, `CK = RR^PP^TT^YY^F1^F2`),
  plus a **1 Hz heartbeat `{0x01,0x01}`** to keep the session alive. F1 flags: 0x01 one-key
  takeoff/land, 0x02 stop, 0x04 calibrate.
- **Failsafe options on this drone:** *hover* = center sticks `(128,128,128,128)` + keep the
  heartbeat going; *land* = one-key land (F1 `0x01`).

> How to DEFINITIVELY confirm the camera format on the Pi:
> `ffprobe -v error -show_streams rtsp://192.168.1.1:7070/webcam` — look at `codec_name`
> (expect `h264`) and `codec_type=video`. If it says `mjpeg`/`rawvideo`, we use the Pi 4's
> hardware H.264 encoder; if H.264, we passthrough.

---

## Open decisions (answer these and the code is unambiguous)
1. Camera format — confirm H.264/RTSP (above) or report what ffprobe says.
2. Command contract — does the desktop send **high-level sticks** (Pi builds the cooingdv
   packet + heartbeat + failsafe) or **raw cooingdv packets** (Pi is a transparent forwarder)?
3. Failsafe action — **hover** or **land** on link loss?
4. Video transport Pi→desktop — **SRT**, plain **UDP/RTP**, or **RTSP** re-serve? (all GStreamer)
</content>
