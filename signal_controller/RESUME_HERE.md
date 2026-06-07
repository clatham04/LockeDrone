# 🚁 Resume Here — Drone WiFi Control

> Pick-up doc. We're **one capture away** from controlling the drone from the Pi.

---

## TL;DR — where we are
- The drone **is** controllable over WiFi (the phone app does it). The Pi is the right tool.
- We found the control channel: **TCP port 7070** on the drone.
- The drone **waits for a handshake** the phone app sends, and **silently resets** when we send the wrong thing. We tried both known packet formats blind — both got reset with zero feedback.
- **The one missing piece:** capture what the app sends on TCP 7070 (the handshake). Then our code works.
- **Tomorrow's job:** do that capture with a Kali Linux USB on the Windows PC (steps below).

---

## Key facts (everything you need)
| Thing | Value |
|------|-------|
| Drone model | EIELEDIY "Quadrotor DIY Kit" — **WiFi UAV** family (app: "DIY Drone") |
| Drone WiFi (SSID) | **FLOW-UFO-2094a5** — open, **single-client** (only one device at a time) |
| Drone IP | **192.168.1.1** |
| Drone MAC | **C6:25:4C:A5:94:20** |
| WiFi channel | **1** (2412 MHz) |
| Pi on drone WiFi | wlan0 = 192.168.1.100 |
| Control channel | **TCP 7070** (open; waits for app handshake, resets on bad input) |
| UDP control ports | all closed (checked 8080, 8800, 7080, 40000, …) |

### ⚠️ The networking gotcha (don't forget)
Your **home network (pfSense) and the drone BOTH use `192.168.1.x`** — a collision. Without forcing traffic out WiFi, packets leak to your home router. Always:
- In code: `socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b"wlan0")` (our scripts do this).
- For tools: `nmap -e wlan0 …`, `ping -I wlan0 …`.
- Allow replies: `sudo sysctl -w net.ipv4.conf.all.rp_filter=2`.

---

## 🌟 BETTER lead (try this FIRST): decompile the app — no Linux, no capture
The TCP 7070 handshake is written *in the app's code*. The drone is a **Lewei "UFO" family**
app (`com.lewei.*`); the "DIY Drone" app you use is one of them. Read the code instead of sniffing:
1. Download the drone's **Android APK** (Android version decompiles easily; same protocol as your iOS app).
2. Open it in **jadx** (free Java decompiler) on the Windows PC.
3. Search for `7070` / socket `connect` → the handshake bytes are right there.
4. Paste the relevant code to Claude → it becomes our `wifi_control.py`.
This is how turbodrone got its protocols. **All software on Windows — no monitor mode, no live USB.**
(Claude can help find the exact APK and read the decompiled code.)

---

## ✅ Fallback plan: capture the handshake (needs monitor-mode WiFi)

> **Capture device:** your ASUS **B650E MAX GAMING WIFI** desktop has onboard **WiFi 6E** (Intel AX210 or MediaTek MT7922 — both do monitor mode on Linux). So the desktop IS the capture device, free.
>
> ⚠️ **WSL will NOT work** — it can't access the real WiFi radio (no monitor mode). You MUST boot a **real Linux Live USB**. ("Try / Live" mode installs nothing; Windows is untouched; reboot back when done.)

> **The capture needs a WiFi adapter in MONITOR mode** (airodump can't capture without it). Two ways:
>
> **Option A — Kali USB, laptop's built-in WiFi (free, but a gamble).** The Alienware's WiFi *might* do monitor mode in Kali (Intel/Killer often do for passive capture, not guaranteed). You find out by running `airmon-ng start wlan0` → `airodump-ng wlan0mon`; if it lists networks, it works. Need: 8 GB+ USB stick + Rufus.
>
> **Option B — $15 USB monitor adapter, capture on the Pi (reliable, simpler).** Buy an **Alfa AWUS036NHA (AR9271)**. Plug into the Pi, set monitor mode, capture on ch 1 — no Kali, no reboot. To avoid killing your SSH, set monitor mode by hand instead of `airmon-ng check kill`:
> ```bash
> sudo ip link set wlan1 down
> sudo iw dev wlan1 set type monitor
> sudo ip link set wlan1 up
> sudo iw dev wlan1 set channel 1
> sudo tcpdump -i wlan1 -w cap.pcap        # then fly the app, Ctrl+C to stop
> ```
> Then read `cap.pcap` (copy to your PC + open in Wireshark, or `tshark -r cap.pcap`).

### Option A steps — Kali USB (on the Windows PC, ~10 min)
- Download **Kali Linux "Live"** ISO: https://www.kali.org/get-kali/ ("Live System").
- Download **Rufus**: https://rufus.ie . Write the ISO to an **8 GB+ USB stick**.

### 2. Boot it
- Reboot PC → spam **F12** → pick the USB → **"Live system"** (nothing installs).

### 3. Capture (in Kali's terminal)
```bash
sudo airmon-ng check kill
sudo airmon-ng                       # note the wifi interface name (e.g. wlan0)
sudo airmon-ng start wlan0           # use that name
sudo airodump-ng -c 1 --bssid C6:25:4C:A5:94:20 -w cap wlan0mon
```

### 4. Make the app talk (iPhone)
- Connect to **FLOW-UFO**, open the DIY Drone app so video loads, then **wiggle the on-screen sticks / tap buttons**.
- 💡 You do **NOT** need to take off — connecting captures the handshake; moving controls captures commands. Props can stay on, drone sitting still.

### 5. Read it
```bash
wireshark cap-01.cap
```
- Filter bar: `tcp.port == 7070`
- Right-click a packet → **Follow → TCP Stream** → set dropdown to **"Hex Dump"**.
- **Copy the first chunk of client→server (red) bytes** — that's the handshake.

### 6. Bring it back
- Paste those bytes to Claude. I'll decode the handshake and update `wifi_control.py`. Then it should fly first try.

---

## If the capture won't work
- `airmon-ng`/`airodump` errors (some laptop WiFi can't do monitor mode) → paste the error.
- Easier alternative: a **~$15 Alfa AWUS036NHA USB WiFi adapter** → same capture, plug-and-play on the Pi (also useful for the autonomous build later).

---

## Files in `signal_controller/` (what each does)
**WiFi path (current):**
- `wifi_control.py` — our control implementation (20-byte `0x66` packet over TCP 7070), `idle`/`takeoff`. Needs the handshake to actually work.
- `native_test.py` — the longer `ef 02 7c` packet variant (blind swing #2).
- `probe_tcp.py` — connect TCP 7070 and print any reply from the drone.
- `capture_app.py` — MITM sniffer (didn't apply: the AP is single-client).

**RF path (abandoned — nRF24):** `test_radio.py`, `rf_scan.py`, `sniff.py`, `bayang.py`, `drone_link.py`, `bind_test.py`. The nRF24 hardware works, but bare-nRF24 XN297/Bayang emulation never bound. Set aside in favor of WiFi.

---

## After we crack control
- Implement clean `connect → handshake → control` in `wifi_control.py` (our code, your readable style).
- Then the fun part: the **anti-drift hold** and **human-follow** you wanted originally (downward camera + ArUco marker for true position hold; YOLO person tracking).

**Bottom line for tomorrow:** Kali USB → `airodump` on channel 1 → fly the app → Wireshark `tcp.port == 7070` → paste the handshake bytes. That's it. 🌙
