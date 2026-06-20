r"""command_relay.py — the COMMAND path:  desktop (eth0) ──► Pi ──► drone (wlan0).

The Pi owns the drone link. It:
  1. LISTENS on eth0 for high-level stick commands from the desktop.
  2. BUILDS the drone's native control packet from them.
  3. SENDS it to the drone on wlan0 at a fixed rate (so the drone always has a fresh command).
  4. Runs the drone's 1 Hz HEARTBEAT.
  5. Consults the FAILSAFE on every send (hover if the desktop goes silent).

The desktop never touches the wire protocol — it just sends sticks.

--- Desktop -> Pi wire format (UDP) ---
    6 bytes:  [roll, pitch, throttle, yaw, flags1, flags2]   (each 0-255, sticks centred 128)

--- Pi -> drone wire format (cooingdv "GL", UDP :7099) ---
    21 bytes: [0x03,0x66,0x14, R,P,T,Y, F1,F2, 0x00*10, CK, 0x99],  CK = R^P^T^Y^F1^F2
    heartbeat: {0x01,0x01}  (~1 Hz)
    F1 flags: 0x01 one-key takeoff/land, 0x02 stop, 0x04 calibrate.
"""
import socket
import threading
import time

CENTER = 128
HEARTBEAT = bytes([0x01, 0x01])
CMD_LEN = 6                                   # desktop command = 6 bytes


def build_gl(roll, pitch, throttle, yaw, flags1=0, flags2=0):
    """Build the 21-byte cooingdv GL control packet."""
    r, p, t, y = int(roll) & 0xFF, int(pitch) & 0xFF, int(throttle) & 0xFF, int(yaw) & 0xFF
    f1, f2 = int(flags1) & 0xFF, int(flags2) & 0xFF
    ck = (r ^ p ^ t ^ y ^ f1 ^ f2) & 0xFF
    return bytes([0x03, 0x66, 0x14, r, p, t, y, f1, f2,
                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ck, 0x99])


def _bind_to_iface(sock, iface):
    """Pin a socket to a NIC (Linux SO_BINDTODEVICE) so drone packets always leave via wlan0.
    Needs root. If it can't, we fall back to normal routing (fine when eth0 is a different
    subnet from the drone's 192.168.1.x)."""
    try:
        opt = getattr(socket, "SO_BINDTODEVICE", 25)
        sock.setsockopt(socket.SOL_SOCKET, opt, iface.encode())
    except (OSError, AttributeError):
        pass


class CommandRelay:
    def __init__(self, cfg, failsafe):
        self.failsafe = failsafe
        self.drone_addr = (cfg["drone"]["ip"], cfg["drone"]["control_port"])
        self.listen_port = cfg["command_listen_port"]
        self.send_hz = cfg["rates"]["command_send_hz"]
        self.hb_hz = cfg["rates"]["heartbeat_hz"]

        # socket to the DRONE (wlan0)
        self._drone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        _bind_to_iface(self._drone, cfg["drone"]["iface"])

        # socket listening for the DESKTOP's commands (eth0)
        self._listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._listen.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen.bind(("0.0.0.0", self.listen_port))
        self._listen.settimeout(0.5)

        self._latest = (CENTER, CENTER, CENTER, CENTER, 0, 0)   # centred until the first packet
        self._latest_ts = 0.0
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._listen_loop, daemon=True).start()
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        print(f"[CMD] desktop -> :{self.listen_port}   |   -> drone {self.drone_addr[0]}:"
              f"{self.drone_addr[1]} @ {self.send_hz} Hz (+ {self.hb_hz} Hz heartbeat)")

    def stop(self):
        self._running = False

    # 1) receive the desktop's commands ------------------------------------------------
    def _listen_loop(self):
        while self._running:
            try:
                data, _ = self._listen.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) >= CMD_LEN:
                with self._lock:
                    self._latest = tuple(data[:CMD_LEN])
                    self._latest_ts = time.time()

    # 2) send to the drone at a fixed rate, failsafe-checked ---------------------------
    def _send_loop(self):
        period = 1.0 / self.send_hz
        while self._running:
            with self._lock:
                latest, ts = self._latest, self._latest_ts
            cmd = self.failsafe.command(latest, ts)        # <-- FAILSAFE: hover if link is silent
            try:
                self._drone.sendto(build_gl(*cmd), self.drone_addr)
            except OSError:
                pass
            time.sleep(period)

    # 3) keep the drone session alive --------------------------------------------------
    def _heartbeat_loop(self):
        period = 1.0 / self.hb_hz
        while self._running:
            try:
                self._drone.sendto(HEARTBEAT, self.drone_addr)
            except OSError:
                pass
            time.sleep(period)
