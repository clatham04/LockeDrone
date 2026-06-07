r"""capture_app.py — sniff the phone app's real control packets (ARP-spoof MITM).

The drone (the Wi-Fi AP) switches traffic, so the Pi can't normally see the phone's
control packets. This ARP-poisons the phone<->drone link so their traffic flows
THROUGH the Pi, where we sniff and print it. That reveals the EXACT UDP port and
packet bytes your app uses — the ground truth we've been guessing at.

Uses scapy (already installed). Run as root.

SETUP
    1. Keep the Pi connected to FLOW-UFO.
    2. Connect your PHONE to FLOW-UFO too, and confirm BOTH stay connected
       (Pi: `ping 192.168.1.1` still works; phone: app video still loads).
    3. Get the phone's IP: iOS Settings > Wi-Fi > (i) next to FLOW-UFO > IP Address.
    4. Let the phone keep flying through the Pi:
           sudo sysctl -w net.ipv4.ip_forward=1
    5. Run it:
           sudo python capture_app.py <PHONE_IP>
    6. Fly with the app — takeoff, move the sticks, land. Watch the printed packets.
       Ctrl+C restores ARP and exits. Paste me the PHONE->DRONE lines.
"""
import sys
import threading
import time

from scapy.all import ARP, Ether, srp, send, sniff, IP, UDP, Raw

IFACE = "wlan0"
DRONE_IP = "192.168.1.1"

_running = True


def mac_of(ip):
    ans, _ = srp(Ether(dst="ff:ff:ff:ff:ff:ff") / ARP(pdst=ip),
                 iface=IFACE, timeout=3, verbose=0)
    for _, r in ans:
        return r.hwsrc
    return None


def poison(phone_ip, phone_mac, drone_mac):
    while _running:
        send(ARP(op=2, pdst=phone_ip, hwdst=phone_mac, psrc=DRONE_IP), iface=IFACE, verbose=0)
        send(ARP(op=2, pdst=DRONE_IP, hwdst=drone_mac, psrc=phone_ip), iface=IFACE, verbose=0)
        time.sleep(2)


def restore(phone_ip, phone_mac, drone_mac):
    for _ in range(5):
        send(ARP(op=2, pdst=phone_ip, hwdst=phone_mac, psrc=DRONE_IP, hwsrc=drone_mac),
             iface=IFACE, verbose=0)
        send(ARP(op=2, pdst=DRONE_IP, hwdst=drone_mac, psrc=phone_ip, hwsrc=phone_mac),
             iface=IFACE, verbose=0)


def show(pkt):
    if UDP in pkt and Raw in pkt:
        src, dst = pkt[IP].src, pkt[IP].dst
        data = bytes(pkt[Raw].load)
        arrow = "PHONE->DRONE" if dst == DRONE_IP else "DRONE->PHONE" if src == DRONE_IP else "----"
        print(f"{arrow:12s} {src}:{pkt[UDP].sport} -> {dst}:{pkt[UDP].dport}  "
              f"len={len(data):3d}  {data.hex()}")


def main():
    if len(sys.argv) < 2:
        print("usage: sudo python capture_app.py <PHONE_IP>")
        return
    phone_ip = sys.argv[1]

    print(f"[CAP] resolving MACs (phone {phone_ip}, drone {DRONE_IP})...")
    phone_mac, drone_mac = mac_of(phone_ip), mac_of(DRONE_IP)
    if not phone_mac or not drone_mac:
        print(f"[CAP] couldn't resolve a MAC (phone={phone_mac}, drone={drone_mac}). "
              "Make sure BOTH the phone and Pi are connected to FLOW-UFO.")
        return

    print(f"[CAP] phone={phone_mac}  drone={drone_mac}")
    print("[CAP] poisoning + sniffing. FLY THE APP NOW (takeoff/move/land). Ctrl+C to stop.\n")
    threading.Thread(target=poison, args=(phone_ip, phone_mac, drone_mac), daemon=True).start()
    try:
        sniff(iface=IFACE, filter=f"udp and host {DRONE_IP}", prn=show, store=0)
    except KeyboardInterrupt:
        pass
    finally:
        global _running
        _running = False
        print("\n[CAP] restoring ARP...")
        restore(phone_ip, phone_mac, drone_mac)


if __name__ == "__main__":
    main()
