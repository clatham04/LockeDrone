#!/usr/bin/env python3
r"""relay.py — Pi 4 bidirectional relay between the drone and the desktop (orchestrator).

  VIDEO   : drone RTSP (wlan0)  ──►  desktop RTP/UDP (eth0)     [video_relay.py]
  COMMAND : desktop (eth0)      ──►  drone cooingdv UDP (wlan0) [command_relay.py]
  SAFETY  : desktop silent > timeout  ──►  hover               [failsafe.py]

The Pi does NOT process images — it forwards video one way and commands the other, with a
mandatory failsafe so a dropped desktop link never leaves the drone uncommanded.

    sudo python3 relay.py        # sudo: SO_BINDTODEVICE on wlan0 needs root
"""
import json
import os
import signal
import time

from command_relay import CommandRelay
from failsafe import Failsafe
from video_relay import VideoRelay

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    with open(os.path.join(HERE, "config.json")) as f:
        cfg = json.load(f)

    fs = cfg["failsafe"]
    failsafe = Failsafe(fs["timeout_s"], fs["action"])
    video = VideoRelay(cfg)                       # VIDEO path  (drone -> desktop)
    command = CommandRelay(cfg, failsafe)         # COMMAND path (desktop -> drone) + failsafe

    print(f"[RELAY] wlan0=drone({cfg['drone']['ip']})  eth0=desktop({cfg['desktop']['ip']})  "
          f"failsafe={fs['action']} @ {fs['timeout_s']}s")
    video.start()
    command.start()
    print("[RELAY] up — forwarding. Ctrl+C to stop.")

    stop = {"flag": False}

    def _sig(*_):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        while not stop["flag"]:
            time.sleep(0.5)
    finally:
        command.stop()
        video.stop()
        print("\n[RELAY] stopped.")


if __name__ == "__main__":
    main()
