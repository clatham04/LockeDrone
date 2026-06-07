r"""wifi_control.py — control the FLOW-UFO drone over Wi-Fi (cooingdv GL protocol).

The KY UFO app = the "cooingdv" drone family; the FLOW- SSID = the "GL" 21-byte
variant. The whole reason TCP 7070 kept resetting us: 7070 is the RTSP *video*
stream (rtsp://192.168.1.1:7070/webcam), NOT control. Control is UDP 7099.

  - Control:   UDP -> 192.168.1.1:7099
  - Heartbeat: {0x01, 0x01} -> :7099 every second (keeps the session alive)

GL control packet (21 bytes):
  03 66 14 RR PP TT YY F1 F2 00*10 CK 99
  RR/PP/TT/YY = roll/pitch/throttle/yaw   (center 128, range 50-200)
  F1: 0x01 one-key takeoff/land, 0x02 stop, 0x04 calibrate, 0x08 flip
  F2: 0x01 headless
  CK = RR ^ PP ^ TT ^ YY ^ F1 ^ F2

We bind the socket to wlan0's IP so packets reach the drone, not the home router
(home + drone both use 192.168.1.x).

USAGE:
  python wifi_control.py calibrate   # gyro cal — LEDs blink, NO props spin (safe test!)
  python wifi_control.py             # idle — centered, motors off
  python wifi_control.py takeoff     # one-key takeoff (drone SECURED, 5s abort)
"""
import fcntl
import socket
import struct
import sys
import threading
import time

DRONE_IP = "192.168.1.1"
CONTROL_PORT = 7099
IFACE = "wlan0"
RATE_HZ = 25
CENTER = 128

HEARTBEAT = bytes([0x01, 0x01])
F1_ONEKEY, F1_STOP, F1_CALIBRATE, F1_FLIP = 0x01, 0x02, 0x04, 0x08
F2_HEADLESS = 0x01

_running = True
_sock = None


def _iface_ip(iface):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(), 0x8915, struct.pack("256s", iface[:15].encode()))[20:24])


def setup():
    """Open the UDP socket, bound to wlan0's IP (dodges the home/drone subnet clash)."""
    global _sock
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _sock.bind((_iface_ip(IFACE), 0))


def build_gl(roll=CENTER, pitch=CENTER, throttle=CENTER, yaw=CENTER, flags1=0, flags2=0):
    p = bytearray(21)
    p[0], p[1], p[2] = 0x03, 0x66, 0x14
    p[3], p[4], p[5], p[6] = roll & 0xFF, pitch & 0xFF, throttle & 0xFF, yaw & 0xFF
    p[7], p[8] = flags1 & 0xFF, flags2 & 0xFF
    # bytes 9..18 stay zero
    p[19] = (roll ^ pitch ^ throttle ^ yaw ^ flags1 ^ flags2) & 0xFF
    p[20] = 0x99
    return bytes(p)


def send(**kw):
    _sock.sendto(build_gl(**kw), (DRONE_IP, CONTROL_PORT))


def _heartbeat():
    while _running:
        try:
            _sock.sendto(HEARTBEAT, (DRONE_IP, CONTROL_PORT))
        except OSError:
            pass
        time.sleep(1.0)


def start():
    threading.Thread(target=_heartbeat, daemon=True).start()
    print(f"[WIFI] heartbeat + control -> {DRONE_IP}:{CONTROL_PORT} "
          f"(UDP from {_sock.getsockname()[0]} via {IFACE})")


def pulse(seconds, **kw):
    period = 1.0 / RATE_HZ
    end = time.time() + seconds
    while time.time() < end:
        send(**kw)
        time.sleep(period)


def stream(label, **kw):
    print(f"[WIFI] {label}  (Ctrl+C to stop)")
    period = 1.0 / RATE_HZ
    n = 0
    try:
        while True:
            send(**kw)
            n += 1
            if n % RATE_HZ == 0:
                print(f"  ...{n}")
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n[WIFI] stopped after {n}")


def run_calibrate():
    start()
    print("[WIFI] gyro-calibration pulse (no props spin — watch for LED blink)...")
    pulse(1.5, flags1=F1_CALIBRATE)
    print("[WIFI] done. Did the drone's LEDs blink / react?")


def run_idle():
    start()
    stream("IDLE — centered, no flags (motors off). Watch the LED.")


def run_takeoff():
    start()
    print("\n" + "!" * 56)
    print("!! ONE-KEY TAKEOFF — props will spin. SECURE THE DRONE.")
    print("!! 5 seconds to abort (Ctrl+C).")
    print("!" * 56 + "\n")
    time.sleep(5)
    try:
        print("[WIFI] takeoff...")
        pulse(0.3, flags1=F1_ONEKEY)        # brief one-key press
        stream("HOVER (centered, altitude-hold). Ctrl+C to land.")
    finally:
        print("[WIFI] landing...")
        pulse(0.3, flags1=F1_ONEKEY)        # one-key again = land


def main():
    global _running
    setup()
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    fn = {"idle": run_idle, "takeoff": run_takeoff, "calibrate": run_calibrate}.get(mode, run_idle)
    try:
        fn()
    finally:
        _running = False


if __name__ == "__main__":
    main()
