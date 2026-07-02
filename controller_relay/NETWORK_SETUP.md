# Network setup — Pi relay on your LAN

The Pi is **dual-homed**: drone on `wlan0`, your home LAN (and the desktop) on `eth0`.
Video flows drone → Pi → desktop; commands flow desktop → Pi → drone. Traffic stays on the LAN.

```
DRONE (wlan0, 192.168.1.1)  ◄──►  PI 4  ◄──►  HOME ROUTER (eth0)  ◄──►  DESKTOP (GPU)
```

## 0. Subnet rule (do this first — it's the usual failure)

The drone's AP is **`192.168.1.x`** and that's fixed in firmware. Your **home LAN must use a
different subnet.** If your router is also `192.168.1.x`, change the router's LAN to something
else (`192.168.0.x`, `10.0.0.x`, …) or the Pi can't tell the two networks apart.

## 1. Pi → drone wifi on wlan0, WITHOUT stealing the default route

The drone AP has no internet. Keep `eth0` as the default route; mark the drone wifi
"never default" so it's used **only** to reach `192.168.1.1`. On Raspberry Pi OS (NetworkManager):

```bash
nmcli dev wifi connect "FLOW-UFO-XXXX"          # <- the drone's SSID (contains "flow")
nmcli con mod "FLOW-UFO-XXXX" ipv4.never-default yes ipv6.never-default yes
nmcli con mod "FLOW-UFO-XXXX" connection.autoconnect yes
```

Verify:
```bash
ip route            # default route must be via eth0; 192.168.1.0/24 dev wlan0 present
ping -c1 192.168.1.1   # drone reachable over wlan0
ping -c1 <desktop-ip>  # desktop reachable over eth0
```

## 2. Find the two IPs and put them in config

- Pi's eth0 IP:    `ip -4 addr show eth0`     → used by the desktop as `PI_IP`
- Desktop's IP:    Windows `ipconfig`         → goes in `config.json` `desktop.ip`

Edit `config.json`:
```json
"desktop": { "ip": "<DESKTOP_HOME_LAN_IP>", "video_port": 5600 }
```
Edit `desktop_stub.py` (or your real desktop app): `PI_IP = "<PI_eth0_IP>"`.

> Tip: give the Pi and desktop **static / DHCP-reserved** IPs in the router so these don't
> change on reboot and break the relay.

## 3. Install deps on the Pi

```bash
sudo apt update
sudo apt install -y python3 gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```

## 4. Test by hand first

```bash
sudo python3 relay.py
```
On the desktop, view the incoming video (also printed by `desktop_stub.py`):
```bash
gst-launch-1.0 -v udpsrc port=5600 \
  caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=false
```
If you see the drone feed on the desktop, the path works.

## 5. Host it as an always-on service

```bash
# adjust WorkingDirectory/ExecStart paths in the unit if you cloned elsewhere
sudo cp lockedrone-relay.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now lockedrone-relay
systemctl status lockedrone-relay
journalctl -u lockedrone-relay -f      # live logs
```
Now the relay starts on boot and restarts itself if the drone wifi blips.

## Desktop side (the GPU brain)

`desktop_stub.py` shows the contract: **receive** RTP/H.264 on `:5600`, **send** 6-byte
sticks `[roll,pitch,throttle,yaw,flags1,flags2]` to the Pi on `:5700`. Swap the dummy hover
for your detector's output. The Pi builds the drone packet, runs the heartbeat, and hovers if
your commands go silent (failsafe).

### Future: Pi's own camera too

The drone keeps its camera (this RTSP path is unchanged). When you add a Pi camera, run a
**second** sender on the Pi to a different `video_port` (e.g. 5602) and a second receiver on
the desktop. The command path doesn't change.
