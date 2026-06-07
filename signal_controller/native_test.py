r"""native_test.py — blind swing #2: the WiFi-UAV "native" 124-byte control frame.

The FLOW- SSID maps to the variant that uses a longer "native" packet (header
ef 02 7c ...) with three rolling 16-bit counters, instead of the short 0x66 frame.
This streams that format over TCP 7070 to see if THIS drone wants it. Reads any reply.

    python native_test.py            # idle (centered, no takeoff) — watch the LED
    python native_test.py takeoff    # spin up (drone SECURED, 5s abort)
"""
import select
import socket
import sys
import time

DRONE_IP = "192.168.1.1"
TCP_PORT = 7070
IFACE = "wlan0"
RATE_HZ = 30
CENTER = 0x80

HEADER          = bytes([0xef, 0x02, 0x7c, 0x00, 0x02, 0x02, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00])
COUNTER1_SUFFIX = bytes([0x00, 0x00, 0x14, 0x00, 0x66, 0x14])
CONTROL_SUFFIX  = bytes(10)
CHECKSUM_SUFFIX = bytes([0x99]) + bytes(44) + bytes([0x32, 0x4b, 0x14, 0x2d, 0x00, 0x00])
COUNTER2_SUFFIX = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                         0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff])
COUNTER3_SUFFIX = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
                         0x00, 0x00, 0x10, 0x00, 0x00, 0x00])

_ctr1, _ctr2, _ctr3 = 0x0000, 0x0001, 0x0002


def build_native(roll=CENTER, pitch=CENTER, throttle=CENTER, yaw=CENTER,
                 takeoff=False, land=False, stop=False, headless=False):
    global _ctr1, _ctr2, _ctr3
    c1 = _ctr1.to_bytes(2, "little")
    c2 = _ctr2.to_bytes(2, "little")
    c3 = _ctr3.to_bytes(2, "little")
    _ctr1 = (_ctr1 + 1) & 0xFFFF
    _ctr2 = (_ctr2 + 1) & 0xFFFF
    _ctr3 = (_ctr3 + 1) & 0xFFFF

    command = 0
    if takeoff or land:
        command |= 0x01
    if stop:
        command |= 0x02
    hdl = 0x03 if headless else 0x02

    # uav axis order: yaw, pitch, throttle, roll
    controls = bytes([yaw & 0xFF, pitch & 0xFF, throttle & 0xFF, roll & 0xFF,
                      command & 0xFF, hdl & 0xFF])
    checksum = 0
    for b in controls:
        checksum ^= b

    pkt = bytearray()
    pkt += HEADER
    pkt += c1 + COUNTER1_SUFFIX
    pkt += controls
    pkt += CONTROL_SUFFIX
    pkt.append(checksum)
    pkt += CHECKSUM_SUFFIX
    pkt += c2 + COUNTER2_SUFFIX
    pkt += c3 + COUNTER3_SUFFIX
    return bytes(pkt)


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "idle"
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, IFACE.encode())
    s.settimeout(5)
    s.connect((DRONE_IP, TCP_PORT))
    print(f"[WIFI] connected {DRONE_IP}:{TCP_PORT} via {IFACE}  (packet len {len(build_native())})")
    s.setblocking(False)

    if mode == "takeoff":
        print("\n!! TAKEOFF — props will spin. SECURE THE DRONE. 5s to abort.\n")
        time.sleep(5)

    period = 1.0 / RATE_HZ
    n = 0
    start = time.time()
    try:
        while True:
            takeoff = mode == "takeoff" and (time.time() - start) < 1.0
            try:
                s.sendall(build_native(takeoff=takeoff))
                n += 1
            except OSError as e:
                print(f"[!!] send failed after {n} packets: {e}  (drone reset us)")
                break
            r, _, _ = select.select([s], [], [], 0.0)
            if r:
                try:
                    data = s.recv(4096)
                except OSError as e:
                    print(f"[!!] recv error after {n} packets: {e}  (drone reset us)")
                    break
                if not data:
                    print(f"[--] drone closed after {n} packets")
                    break
                print(f"[<<] drone replied {len(data)} bytes: {data[:96].hex()}")
            if n % RATE_HZ == 0:
                print(f"  ...{n} packets")
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\n[WIFI] stopped after {n} packets")
    finally:
        s.close()


if __name__ == "__main__":
    main()
