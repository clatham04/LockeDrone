r"""wifi_control.py — control the FLOW-UFO drone over Wi-Fi (our own implementation).

The control packet is the WiFi-UAV family's 20-byte frame (the byte layout is just
what the drone expects — facts, like a datasheet). The reference drones take it over
UDP, but THIS drone has its UDP control ports closed and TCP 7070 open and waiting —
so we open 7070 and stream our packets over it. Figuring that out is the part that's
ours.

Packet (20 bytes):
    66 SP RR PP TT YY FL 0a 00*10 CK 99
    SP speed (0x14)   RR roll  PP pitch  TT throttle  YY yaw   (0x80 = center)
    FL flags: 0x01 takeoff, 0x02 land, 0x04 stop
    CK checksum = XOR of bytes[2..17]      99 footer

SAFETY: a plain run sends IDLE (centered sticks, no takeoff) -> motors stay off.
Secure the drone anyway (props on).
    python wifi_control.py            # idle — watch the LED
    python wifi_control.py takeoff    # spin up (drone SECURED, 5s abort)
"""
import socket
import sys
import time

DRONE_IP = "192.168.1.1"
TCP_PORT = 7070          # the open port your drone waits on
IFACE = "wlan0"          # force traffic out wifi (home + drone both use 192.168.1.x)
RATE_HZ = 30
SPEED = 0x14
CENTER = 0x80


def build_packet(roll=CENTER, pitch=CENTER, throttle=CENTER, yaw=CENTER,
                 takeoff=False, land=False, stop=False):
    """Build one 20-byte WiFi-UAV control frame."""
    p = bytearray(20)
    p[0] = 0x66
    p[1] = SPEED
    p[2] = roll & 0xFF
    p[3] = pitch & 0xFF
    p[4] = throttle & 0xFF
    p[5] = yaw & 0xFF
    p[6] = (0x01 if takeoff else 0) | (0x02 if land else 0) | (0x04 if stop else 0)
    p[7] = 0x0A
    # bytes 8..17 stay zero
    chk = 0
    for i in range(2, 18):
        chk ^= p[i]
    p[18] = chk & 0xFF
    p[19] = 0x99
    return bytes(p)


def connect_tcp():
    """Open the drone's control connection, forced out wlan0."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, IFACE.encode())
    s.settimeout(5)
    s.connect((DRONE_IP, TCP_PORT))
    print(f"[WIFI] connected to drone {DRONE_IP}:{TCP_PORT} via {IFACE}")
    return s


def stream(sock, make_packet, label):
    """Send a packet at RATE_HZ until Ctrl+C."""
    print(f"[WIFI] {label}  (Ctrl+C to stop)")
    period = 1.0 / RATE_HZ
    n = 0
    try:
        while True:
            sock.sendall(make_packet())
            n += 1
            if n % RATE_HZ == 0:
                print(f"  ...{n} packets")
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n[WIFI] stopped after {n} packets")


def run_idle(sock):
    stream(sock, build_packet, "IDLE — centered, no takeoff (motors off). Watch the LED.")


def run_takeoff(sock):
    print("\n" + "!" * 56)
    print("!! TAKEOFF — props WILL spin. SECURE THE DRONE.")
    print("!! 5 seconds to abort (Ctrl+C).")
    print("!" * 56 + "\n")
    time.sleep(5)
    period = 1.0 / RATE_HZ
    try:
        print("[WIFI] takeoff...")
        end = time.time() + 1.0
        while time.time() < end:
            sock.sendall(build_packet(takeoff=True))
            time.sleep(period)
        stream(sock, build_packet, "HOVER (throttle center). Ctrl+C to land.")
    finally:
        print("[WIFI] landing...")
        for _ in range(20):
            sock.sendall(build_packet(land=True))
            time.sleep(period)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    sock = connect_tcp()
    try:
        (run_takeoff if mode == "takeoff" else run_idle)(sock)
    finally:
        sock.close()
        print("[WIFI] connection closed")


if __name__ == "__main__":
    main()
