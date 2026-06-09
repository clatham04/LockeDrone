r"""wifi_connect.py — find and connect to the drone's WiFi (Windows).

The drone's access point always has a keyword in its name (default "flow", e.g.
FLOW-UFO-2094a5). This scans for a matching network and connects via netsh.

Run as Administrator. Standalone:
    python wifi_connect.py            # uses keyword "flow"
    python wifi_connect.py flow       # explicit keyword
"""
import subprocess
import sys
import time


def _ssids():
    """Return a list of visible SSIDs using netsh."""
    out = subprocess.run(
        ["netsh", "wlan", "show", "networks"],
        capture_output=True, text=True
    )
    ssids = []
    for line in out.stdout.splitlines():
        line = line.strip()
        if line.startswith("SSID") and "BSSID" not in line:
            # line looks like: "SSID 1 : FLOW-UFO-2094a5"
            parts = line.split(":", 1)
            if len(parts) == 2:
                ssid = parts[1].strip()
                if ssid:
                    ssids.append(ssid)
    return ssids


def _current_ssid():
    """Return the SSID we're currently connected to, or None."""
    out = subprocess.run(
        ["netsh", "wlan", "show", "interfaces"],
        capture_output=True, text=True
    )
    for line in out.stdout.splitlines():
        line = line.strip()
        if line.startswith("SSID") and "BSSID" not in line:
            parts = line.split(":", 1)
            if len(parts) == 2:
                return parts[1].strip()
    return None


def find_and_connect(name_contains="flow"):
    """Scan for an SSID containing `name_contains` and connect. Returns the SSID or None."""
    # check if we're already connected to it
    current = _current_ssid()
    if current and name_contains.lower() in current.lower():
        print(f"[WIFI] already connected to '{current}'")
        return current

    # scan
    matches = [s for s in _ssids() if name_contains.lower() in s.lower()]
    if not matches:
        print(f"[WIFI] no network with '{name_contains}' in the name. Is the drone on?")
        return None

    ssid = matches[0]
    print(f"[WIFI] found '{ssid}' — connecting...")

    # netsh connect (the network profile must exist — Windows saves it after first manual connect)
    r = subprocess.run(
        ["netsh", "wlan", "connect", f"name={ssid}"],
        capture_output=True, text=True
    )
    if r.returncode != 0 or "error" in r.stdout.lower():
        print(f"[WIFI] connect failed: {r.stdout.strip()}")
        print("       Tip: connect to the drone WiFi manually once in Windows settings,")
        print("       then this script can reconnect automatically next time.")
        return None

    # give Windows a moment to associate
    time.sleep(2)

    current = _current_ssid()
    if current and name_contains.lower() in current.lower():
        return current

    print(f"[WIFI] connected command sent but association unconfirmed — continuing anyway.")
    return ssid


if __name__ == "__main__":
    keyword = sys.argv[1] if len(sys.argv) > 1 else "flow"
    ssid = find_and_connect(keyword)
    print(f"connected to {ssid}" if ssid else "no matching network found")