r"""probe_tcp.py — connect to the drone's open TCP port and see what it sends.

The drone listens on TCP 7070 (the only open control-ish port; all UDP candidates
are closed). This connects via wlan0 and prints whatever the drone sends, to learn
how the control protocol starts.

    python probe_tcp.py
"""
import socket

DRONE_IP = "192.168.1.1"
PORT = 7070

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# force out wlan0 (home + drone share 192.168.1.x)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b"wlan0")
s.settimeout(8)

try:
    s.connect((DRONE_IP, PORT))
    print(f"[OK] connected to drone {DRONE_IP}:{PORT}")
    try:
        while True:
            data = s.recv(2048)
            if not data:
                print("[--] drone closed the connection")
                break
            print(f"[<<] {len(data)} bytes: {data.hex()}")
    except socket.timeout:
        print("[--] drone sent nothing in 8s (it's probably waiting for US to send first)")
finally:
    s.close()
