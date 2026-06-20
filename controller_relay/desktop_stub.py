r"""desktop_stub.py — minimal DESKTOP side, showing the exact contract with the Pi relay.

Runs on the desktop (the GPU brain). It demonstrates BOTH halves of the contract:
  - RECEIVE: the drone video arrives as RTP/H.264 on UDP :VIDEO_PORT. Decode it with your
    detector. (This stub just prints the GStreamer line to view it.)
  - SEND:    6-byte stick commands to the Pi's :COMMAND_PORT. The Pi turns them into the
    drone's native packet, runs the heartbeat, and applies the failsafe.

>> This is a STUB. Replace the dummy hover command in the loop with your detection output. <<

Command wire format (must match command_relay.py): 6 bytes
    [roll, pitch, throttle, yaw, flags1, flags2]   each 0-255, sticks centred at 128
    flags1: 0x01 one-key takeoff/land, 0x02 stop, 0x04 calibrate
"""
import socket
import time

PI_IP = "192.168.2.1"          # the Pi's eth0 address (where the Pi listens for commands)
COMMAND_PORT = 5700            # must match config.json -> command_listen_port
VIDEO_PORT = 5600             # must match config.json -> desktop.video_port
CENTER = 128
SEND_HZ = 25

VIEW_VIDEO = (
    f'gst-launch-1.0 -v udpsrc port={VIDEO_PORT} '
    f'caps="application/x-rtp,media=video,encoding-name=H264,payload=96" '
    f'! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=false'
)


def send_command(sock, roll, pitch, throttle, yaw, flags1=0, flags2=0):
    """Send ONE stick command to the Pi (it builds the drone packet + heartbeat + failsafe)."""
    sock.sendto(bytes([roll & 0xFF, pitch & 0xFF, throttle & 0xFF,
                       yaw & 0xFF, flags1 & 0xFF, flags2 & 0xFF]), (PI_IP, COMMAND_PORT))


def main():
    print(f"[DESKTOP] view the drone video with:\n    {VIEW_VIDEO}\n")
    print(f"[DESKTOP] sending HOVER sticks to {PI_IP}:{COMMAND_PORT} at {SEND_HZ} Hz "
          f"(replace with your detection output)")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / SEND_HZ
    try:
        while True:
            # ── your detector runs here: receive video on VIDEO_PORT, detect, decide sticks ──
            roll, pitch, throttle, yaw, f1, f2 = CENTER, CENTER, CENTER, CENTER, 0, 0
            send_command(sock, roll, pitch, throttle, yaw, f1, f2)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[DESKTOP] stopped (Pi will hover after the failsafe timeout).")


if __name__ == "__main__":
    main()
