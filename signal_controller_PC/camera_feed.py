r"""camera_feed.py — pull the drone's forward camera (RTSP) on Windows.

The drone's video is an RTSP stream at rtsp://192.168.1.1:7070/webcam.
Saves a snapshot to drone_snapshot.jpg and prints resolution + frame rate.

Run as Administrator (needed for route add):
    python camera_feed.py
"""
import subprocess
import time

import cv2

URL = "rtsp://192.168.1.1:7070/webcam"
FRAMES = 100


def ensure_route():
    """Pin the drone's IP via a host route on Windows."""
    subprocess.run(
        ["route", "add", "192.168.1.1", "mask", "255.255.255.255", "192.168.1.1"],
        capture_output=True
    )


def main():
    ensure_route()
    print(f"[CAM] opening {URL} ...")
    cap = cv2.VideoCapture(URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[CAM] could NOT open the stream.")
        print("      Check: drone ON? connected to FLOW-UFO wifi?")
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
    else:
        print("[CAM] no frames decoded.")
    cap.release()


if __name__ == "__main__":
    main()