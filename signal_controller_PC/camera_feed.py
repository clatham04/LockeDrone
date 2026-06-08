r"""camera_feed.py — pull the drone's forward camera (RTSP) on the Pi.

The drone's video is an RTSP stream at rtsp://192.168.1.1:7070/webcam. This is the
foundation for ANY vision (drift estimation, follow, etc.) — first we just confirm
the Pi can actually see it.

SUBNET COLLISION: home + drone both use 192.168.1.x, so first force traffic to the
drone out wlan0 with a host route:

    sudo ip route add 192.168.1.1/32 dev wlan0
    # ...run this script...
    sudo ip route del 192.168.1.1/32 dev wlan0     # undo it afterward

Then:
    python camera_feed.py

Headless-friendly: it saves a snapshot to disk (drone_snapshot.jpg) and prints the
resolution + frame rate, so you don't need a display over SSH.
"""
import time

import cv2

URL = "rtsp://192.168.1.1:7070/webcam"
FRAMES = 100


def main():
    print(f"[CAM] opening {URL} ...")
    cap = cv2.VideoCapture(URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[CAM] could NOT open the stream.")
        print("      Most likely the subnet route — run this first:")
        print("        sudo ip route add 192.168.1.1/32 dev wlan0")
        return

    print(f"[CAM] stream open — grabbing {FRAMES} frames...")
    n = 0
    t0 = time.time()
    last = None
    while n < FRAMES:
        ok, frame = cap.read()
        if not ok:
            print(f"[CAM] read failed at frame {n}")
            break
        last = frame
        n += 1
        if n % 20 == 0:
            print(f"  ...{n} frames  ({frame.shape[1]}x{frame.shape[0]})")

    elapsed = time.time() - t0
    fps = n / elapsed if elapsed > 0 else 0
    if last is not None:
        cv2.imwrite("drone_snapshot.jpg", last)
        print(f"[CAM] saved drone_snapshot.jpg  {last.shape[1]}x{last.shape[0]}  ~{fps:.1f} FPS")
        print("      (scp it to your PC to view, or it confirms the feed works.)")
    else:
        print("[CAM] no frames decoded.")
    cap.release()


if __name__ == "__main__":
    main()
