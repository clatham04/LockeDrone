r"""wifi_control.py — control the FLOW-UFO drone over Wi-Fi (UDP).

Ported from the working WiFi_UFO protocol (github.com/LukasMaly/wifi-ufo-drone).
The key insight: the drone needs a constant HEARTBEAT to keep the control link
alive before it will accept any control packets. That's why sending control blind
did nothing.

Link (edit to match your network):
    drone   192.168.1.1   via wlan0
    TCP heartbeat -> drone:7060    bare SYNs ~20 Hz   (needs scapy + root)
    UDP heartbeat -> drone:40000   [63 63 01 00 00 00 00] @ 1 Hz
    UDP control   -> drone:40000   15-byte packet ~20 Hz

Control packet:
    63 63 0a 00 00 08 00 66 | RR PP TT YY | MM | XX | 99
    RR roll, PP pitch, TT throttle (0=off, 128=hover), YY yaw   (128 = center)
    MM mode: 0 none, 1 take off, 2 land, 4 stop
    XX checksum = RR ^ PP ^ TT ^ YY ^ MM

SAFETY: running it plain only starts the heartbeat and holds IDLE (throttle 0,
mode 0) — no takeoff. Secure the drone anyway (props on). Takeoff is a separate,
explicit command we add once the link is confirmed.

    sudo python wifi_control.py          # link + idle, watch the LED  (sudo for scapy)
"""
import random
import socket
import threading
import time

# TCP heartbeat needs raw SYNs (scapy). Degrade gracefully to UDP-only if missing.
try:
    from scapy.all import IP, TCP, sr1
    HAVE_SCAPY = True
except ImportError:
    HAVE_SCAPY = False

DRONE_IP = "192.168.1.1"
IFACE = "wlan0"
TCP_PORT = 7060
UDP_PORT = 40000

UDP_HEARTBEAT = bytes([0x63, 0x63, 0x01, 0x00, 0x00, 0x00, 0x00])
FLY_TEMPLATE = bytearray([0x63, 0x63, 0x0A, 0x00, 0x00, 0x08, 0x00, 0x66,
                          0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x99])

_running = True
_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def build_cmd(roll=128, pitch=128, throttle=0, yaw=128, mode=0):
    p = FLY_TEMPLATE[:]
    p[8], p[9], p[10], p[11], p[12] = roll, pitch, throttle, yaw, mode
    p[13] = p[8] ^ p[9] ^ p[10] ^ p[11] ^ p[12]      # checksum over the 5 data bytes
    return bytes(p)


def cmd(roll=128, pitch=128, throttle=0, yaw=128, mode=0):
    _udp.sendto(build_cmd(roll, pitch, throttle, yaw, mode), (DRONE_IP, UDP_PORT))


def _tcp_heartbeat():
    while _running:
        syn = IP(dst=DRONE_IP, ttl=63) / TCP(sport=random.randint(32768, 49152),
                                             dport=TCP_PORT, flags="S", seq=0)
        sr1(syn, iface=IFACE, timeout=0.05, verbose=0)
        time.sleep(0.05)


def _udp_heartbeat():
    while _running:
        _udp.sendto(UDP_HEARTBEAT, (DRONE_IP, UDP_PORT))
        time.sleep(1.0)


def connect():
    """Start the heartbeats that keep the control link alive. Call before cmd()."""
    threading.Thread(target=_udp_heartbeat, daemon=True).start()
    if HAVE_SCAPY:
        threading.Thread(target=_tcp_heartbeat, daemon=True).start()
        print(f"[WIFI] Heartbeats up — TCP {DRONE_IP}:{TCP_PORT} + UDP {DRONE_IP}:{UDP_PORT}.")
    else:
        print(f"[WIFI] UDP heartbeat only ({DRONE_IP}:{UDP_PORT}). "
              f"Install scapy + run as sudo for the TCP heartbeat if the drone ignores us.")


def _stream(make_packet_args, label, hz=20):
    """Stream control packets (a dict of cmd() kwargs) until Ctrl+C."""
    print(f"[WIFI] {label}  (Ctrl+C to stop)")
    sent = 0
    period = 1.0 / hz
    try:
        while True:
            cmd(**make_packet_args)
            sent += 1
            if sent % hz == 0:
                print(f"  ...{sent}")
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n[WIFI] Stopped after {sent} packets.")


def _send_for(seconds, **kwargs):
    end = time.time() + seconds
    while time.time() < end:
        cmd(**kwargs)
        time.sleep(0.05)


def run_idle():
    connect()
    _stream({}, "IDLE — throttle 0, no takeoff (motors stay off). Watch the LED.")


def run_takeoff():
    connect()
    print("\n" + "!" * 60)
    print("!! TAKEOFF — THE MOTORS/PROPS WILL SPIN UP.")
    print("!! SECURE THE DRONE (tape/strap it down) and keep clear.")
    print("!! Starting in 5 seconds. Ctrl+C now to abort.")
    print("!" * 60 + "\n")
    time.sleep(5)
    global _running
    try:
        print("[WIFI] takeoff...")
        _send_for(1.0, mode=1)                 # mode 1 = take off
        print("[WIFI] holding (altitude-hold hover). Ctrl+C to land.")
        while True:
            cmd(throttle=128, mode=0)          # 128 = hold altitude
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[WIFI] landing + cutting motors...")
        _send_for(1.0, mode=2)                 # land
        _send_for(0.5, mode=4)                 # stop (hard cut)
        _running = False


def run_land():
    connect()
    _send_for(1.5, mode=2)
    print("[WIFI] land command sent.")


def run_stop():
    connect()
    _send_for(1.0, mode=4)
    print("[WIFI] stop (motor cut) sent.")


def main():
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    {"idle": run_idle, "takeoff": run_takeoff,
     "land": run_land, "stop": run_stop}.get(mode, run_idle)()


if __name__ == "__main__":
    main()
