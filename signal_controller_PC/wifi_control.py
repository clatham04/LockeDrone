r"""wifi_control.py — control the FLOW-UFO drone over Wi-Fi (cooingdv GL protocol).

Windows-compatible version. Removed fcntl / SO_BINDTODEVICE (Linux-only).
Socket is bound to the local WiFi IP detected automatically.

  - Control:   UDP -> 192.168.1.1:7099
  - Heartbeat: {0x01, 0x01} -> :7099 every second (keeps the session alive)

GL control packet (21 bytes):
  03 66 14 RR PP TT YY F1 F2 00*10 CK 99
  RR/PP/TT/YY = roll/pitch/throttle/yaw   (center 128, range 50-200)
  F1: 0x01 one-key takeoff/land, 0x02 stop, 0x04 calibrate, 0x08 flip
  F2: 0x01 headless
  CK = RR ^ PP ^ TT ^ YY ^ F1 ^ F2

USAGE:
  python wifi_control.py calibrate   # gyro cal — LEDs blink, NO props spin (safe test!)
  python wifi_control.py             # idle — centered, motors off
  python wifi_control.py takeoff     # one-key takeoff (drone SECURED, 5s abort)
  python wifi_control.py stop        # emergency motor cut
"""
import socket
import sys
import threading
import time

DRONE_IP = "192.168.1.1"
CONTROL_PORT = 7099
RATE_HZ = 25
CENTER = 128

HEARTBEAT = bytes([0x01, 0x01])
F1_ONEKEY, F1_STOP, F1_CALIBRATE, F1_FLIP = 0x01, 0x02, 0x04, 0x08
F2_HEADLESS = 0x01

_running = True
_sock = None


def _local_ip():
    """Find the local IP on the drone's subnet (192.168.1.x) by connecting a dummy socket."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((DRONE_IP, CONTROL_PORT))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "0.0.0.0"


def setup():
    """Open the UDP socket bound to the WiFi interface facing the drone."""
    global _sock
    local_ip = _local_ip()
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _sock.bind((local_ip, 0))
    print(f"[WIFI] socket bound to {local_ip}")


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
    print(f"[WIFI] heartbeat + control -> {DRONE_IP}:{CONTROL_PORT}")


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


def run_stop():
    """EMERGENCY motor cut — drops the drone immediately."""
    start()
    print("[WIFI] *** EMERGENCY STOP *** cutting motors...")
    pulse(1.0, flags1=F1_STOP)
    print("[WIFI] stop sent.")


def run_takeoff():
    start()
    print("\n" + "!" * 56)
    print("!! ONE-KEY TAKEOFF — props will spin. SECURE THE DRONE.")
    print("!! 5 seconds to abort (Ctrl+C).")
    print("!" * 56 + "\n")
    time.sleep(5)
    try:
        print("[WIFI] takeoff...")
        pulse(0.3, flags1=F1_ONEKEY)
        stream("HOVER (centered, altitude-hold). Ctrl+C to land.")
    finally:
        print("[WIFI] landing...")
        pulse(0.3, flags1=F1_ONEKEY)


def main():
    global _running
    setup()
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    fn = {"idle": run_idle, "takeoff": run_takeoff, "calibrate": run_calibrate,
          "stop": run_stop}.get(mode, run_idle)
    try:
        fn()
    finally:
        _running = False


if __name__ == "__main__":
    main()