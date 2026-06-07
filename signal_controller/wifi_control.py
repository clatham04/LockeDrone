"""wifi_control.py — control the FLOW-UFO drone over Wi-Fi (UDP).

The drone is a WiFi_UFO / FLOW-UFO family quad. Control is a 14-byte UDP packet
sent ~every 40 ms. Packet format (decoded from the WiFi_UFO protocol):

    cc 5a 01 83 09 66 | RR PP TT YY | FF | XX | 99 74
    \---- header ----/   \channels/  flag csum \footer/

    RR roll, PP pitch, TT throttle, YY yaw   (0x80 = center)
    FF flags (takeoff/land/etc.)
    XX checksum = RR ^ PP ^ TT ^ YY ^ FF

SAFETY: run with no arguments and it ONLY sends the IDLE packet (all centers, no
takeoff) — on an altitude-hold drone that keeps the motors OFF on the ground, just
like the app sitting idle. Takeoff/flight are separate, explicit actions. Secure
the drone anyway, since the props are on and we're confirming an inferred protocol.

    python wifi_control.py            # idle only (safe) — watch the drone's LED
"""
import socket
import time

DRONE_IP = "192.168.1.1"
DRONE_PORT = 7080                    # WiFi_UFO control port (adjust if recon says otherwise)

HEADER = bytes([0xCC, 0x5A, 0x01, 0x83, 0x09, 0x66])
FOOTER = bytes([0x99, 0x74])
CENTER = 0x80

# Flag bits (to be confirmed by capture before we ever use takeoff):
FLAG_NONE = 0x00


def build_packet(roll=CENTER, pitch=CENTER, throttle=CENTER, yaw=CENTER, flags=FLAG_NONE):
    body = bytes([roll, pitch, throttle, yaw, flags])
    checksum = 0
    for b in body:
        checksum ^= b
    return HEADER + body + bytes([checksum]) + FOOTER


def stream(packet, label, hz=25):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / hz
    print(f"Sending {label} to {DRONE_IP}:{DRONE_PORT} at {hz} Hz. Ctrl+C to stop.")
    print(f"packet = {packet.hex()}")
    sent = 0
    try:
        while True:
            sock.sendto(packet, (DRONE_IP, DRONE_PORT))
            sent += 1
            if sent % hz == 0:
                print(f"  ...{sent} packets sent")
            time.sleep(period)
    except KeyboardInterrupt:
        print(f"\nStopped after {sent} packets.")
    finally:
        sock.close()


def main():
    # IDLE ONLY: centered sticks, no takeoff flag -> motors stay off on the ground.
    idle = build_packet()
    stream(idle, "IDLE packet (no takeoff — motors stay off)")


if __name__ == "__main__":
    main()
