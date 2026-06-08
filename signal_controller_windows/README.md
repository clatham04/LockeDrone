# signal_controller_windows — drone camera detection on a Windows PC

Run the forward-camera person detection on **Windows** (bigger, smoother view than the
Pi's remote desktop). **No flying** — this is just the detection preview. The drone's
actual flight + human-follow runs on the **Pi** ([`../signal_controller/`](../signal_controller/)).

> Same idea as the Pi's `detect_test.py`, with the Linux-only bits (the `wlan0` route,
> `ip`/`ping -I`) removed and the `.pt` model used instead of ncnn (rock-solid on Windows).

---

## One-time setup

**1. Install ffmpeg** (the camera decodes through it):
```powershell
winget install Gyan.FFmpeg
```
Then **open a new terminal** and confirm:
```powershell
ffmpeg -version
```

**2. Install the Python deps** (in a venv):
```powershell
cd "<...>\LockeDrone\signal_controller_windows"
py -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```
> `ultralytics` pulls in PyTorch — a large download. Let it finish.

---

## Run it

**1. Connect this PC to the drone's wifi** — `FLOW-UFO-…`.
- **Disconnect from your home wifi / unplug ethernet.** Both your home network and the
  drone use `192.168.1.x`; if you're on both, Windows sends the camera traffic to the
  wrong one. Being on **only** the drone wifi avoids that.
- Make sure the **drone is ON** and your **phone is disconnected** from `FLOW-UFO`
  (it's single-client).

**2. Start detection:**
```powershell
python detect_test.py
```
A window opens with green boxes around detected people. **Press `q` to quit.**

---

## Troubleshooting
| Symptom | Fix |
|---|---|
| `can't reach the drone at 192.168.1.1` | You're not on (only) `FLOW-UFO`. Disconnect home wifi/ethernet, reconnect to the drone, phone off it. |
| `ffmpeg not found on PATH` | `winget install Gyan.FFmpeg`, then open a **new** terminal. |
| `no video after 15s` | Drone on + streaming? Try the manual test: `ffmpeg -rtsp_transport udp -i rtsp://192.168.1.1:7070/webcam -frames:v 1 -f null -` |
| Low FPS | Lower `IMGSZ` in `detect_test.py` (e.g. 320). |
