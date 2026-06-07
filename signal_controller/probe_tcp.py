r"""probe_tcp.py — send to the drone's TCP 7070 and print anything it sends back.

The drone resets 7070 when we send our control packets, so it's parsing (and
rejecting) them. This sends a few packets and reads any reply — an error/ack/handshake
hint that tells us what it actually wants.

    python probe_tcp.py
"""
import select
import socket
import time

DRONE_IP = "192.168.1.1"
PORT = 7070


def idle_packet():
    p = bytearray(20)
    p[0] = 0x66
    p[1] = 0x14
    p[2] = p[3] = p[4] = p[5] = 0x80
    p[7] = 0x0A
    chk = 0
    for i in range(2, 18):
        chk ^= p[i]
    p[18] = chk & 0xFF
    p[19] = 0x99
    return bytes(p)


def main():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b"wlan0")
    s.settimeout(3)
    s.connect((DRONE_IP, PORT))
    print(f"[OK] connected {DRONE_IP}:{PORT}")
    s.setblocking(False)

    sent = 0
    start = time.time()
    try:
        while time.time() - start < 8:
            try:
                s.sendall(idle_packet())
                sent += 1
            except OSError as e:
                print(f"[!!] send failed after {sent} packets: {e}")
                break
            # any reply?
            r, _, _ = select.select([s], [], [], 0.05)
            if r:
                try:
                    data = s.recv(4096)
                except OSError as e:
                    print(f"[!!] recv error: {e}")
                    break
                if not data:
                    print(f"[--] drone closed the connection after we sent {sent} packets")
                    break
                print(f"[<<] drone replied {len(data)} bytes: {data[:96].hex()}")
            time.sleep(0.03)
    finally:
        print(f"[--] done, sent {sent} packets")
        s.close()


if __name__ == "__main__":
    main()
