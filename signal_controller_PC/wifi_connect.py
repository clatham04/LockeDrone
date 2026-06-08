r"""wifi_connect.py — find and connect to the drone's WiFi.

The drone's access point always has a keyword in its name (default "flow", e.g.
FLOW-UFO-2094a5). This scans for a matching network and connects via NetworkManager.

Run with sudo (nmcli connect needs it). Standalone:
    sudo python3 wifi_connect.py            # uses keyword "flow"
    sudo python3 wifi_connect.py flow        # explicit keyword
"""
import subprocess
import sys


def _ssids():
    out = subprocess.run(["nmcli", "-t", "-f", "SSID", "device", "wifi", "list"],
                         capture_output=True, text=True)
    return [s for s in out.stdout.splitlines() if s.strip()]


def find_and_connect(name_contains="flow"):
    """Scan for an SSID containing `name_contains` and connect. Returns the SSID or None."""
    subprocess.run(["nmcli", "device", "wifi", "rescan"], capture_output=True)
    matches = [s for s in _ssids() if name_contains.lower() in s.lower()]
    if not matches:
        print(f"[WIFI] no network with '{name_contains}' in the name. Is the drone on?")
        return None

    ssid = matches[0]
    print(f"[WIFI] found '{ssid}' — connecting...")
    r = subprocess.run(["nmcli", "device", "wifi", "connect", ssid],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[WIFI] connect failed: {r.stderr.strip()}")
        return None
    return ssid


if __name__ == "__main__":
    keyword = sys.argv[1] if len(sys.argv) > 1 else "flow"
    ssid = find_and_connect(keyword)
    print(f"connected to {ssid}" if ssid else "no matching network connected")
