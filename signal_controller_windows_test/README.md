# signal_controller_windows_test — drone camera detection on a Windows PC

Run the forward-camera person detection on **Windows** (bigger, smoother view than the
Pi's remote desktop). No flying — this is just the detection preview.

> The Pi version lives in [`../signal_controller/`](../signal_controller/). This folder is
> the same idea with the Linux-only bits removed, and — since the PC has the headroom the
> Pi lacks — a **bigger model** (`yolo11m.pt`) at **full resolution**, running every frame,
> so it detects smaller / farther / partially-hidden people.

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
cd "<...>\LockeDrone\signal_controller_windows_test"
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
A window opens with green boxes around detected people. **Press `q` to quit.** The
startup line shows the model + whether it's running `on GPU (CUDA)` or `on CPU`.

---

## Detection quality (tuned to "see more")
Set in `detect_test.py`:
- **`MODEL = "yolo11m.pt"`** — far stronger than the Pi's nano. Bump to `yolo11l.pt` or
  `yolo11x.pt` for even better (downloads automatically on first run).
- **`IMGSZ = 640`** — full stream resolution, so smaller/farther people get picked up.
- **`CONF = 0.30`** — lower confidence threshold catches more (raise it if you get false boxes).
- **GPU = big speed-up:** the default `pip install` gives **CPU-only** PyTorch. With an
  NVIDIA GPU, switch to the CUDA build so the big models fly:
  ```powershell
  pip uninstall -y torch torchvision
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
  python -c "import torch; print(torch.cuda.is_available())"   # should print: True
  ```

---

## Troubleshooting
| Symptom | Fix |
|---|---|
| `can't reach the drone at 192.168.1.1` | You're not on (only) `FLOW-UFO`. Disconnect home wifi/ethernet, reconnect to the drone, phone off it. |
| `ffmpeg not found on PATH` | `winget install Gyan.FFmpeg`, then open a **new** terminal. |
| `no video after 15s` | Drone on + streaming? Try the manual test: `ffmpeg -rtsp_transport udp -i rtsp://192.168.1.1:7070/webcam -frames:v 1 -f null -` |
| Low FPS on CPU | Use a smaller `MODEL` (`yolo11s.pt`/`yolo11n.pt`) or lower `IMGSZ`; or install GPU PyTorch. |
