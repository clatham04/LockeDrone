"""rf_scan.py — is the nRF24 receiving ANYTHING at all? (protocol-free hardware check)

Sweeps all 126 2.4 GHz channels and uses the radio's Received Power Detector (RPD)
to see if any RF energy is present — WiFi, Bluetooth, microwaves, your drone
controller, etc. This decides whether our problem is the RADIO or the PROTOCOL.

    python rf_scan.py

Reading the result:
  - Activity on many channels (a WiFi cluster, etc.) -> the nRF24 RX WORKS, so the
    XN297 sniff failure is a protocol/alignment issue, not hardware.
  - All zeros everywhere -> the radio isn't receiving at all: bad module, antenna,
    or power/brownout (the capacitor!). That's the bug.

Make sure there's traffic to find: keep your phone's WiFi/Bluetooth on nearby, and
turn the stock controller ON too. 2.4 GHz WiFi shows up around channels ~2-80.
"""
import time

from pyrf24 import RF24, RF24_1MBPS

CE_PIN = 22
CSN_PIN = 0
CHANNELS = 126
SWEEPS = 100          # passes over all channels; more = more sensitive (slower)


def _rpd(radio):
    """Read the Received Power Detector, tolerant of either method name."""
    try:
        return radio.testRPD()
    except AttributeError:
        return radio.testCarrier()


def main():
    radio = RF24(CE_PIN, CSN_PIN)
    if not radio.begin():
        raise RuntimeError("nRF24 not responding — run test_radio.py first.")

    radio.setAutoAck(False)
    radio.setDataRate(RF24_1MBPS)
    radio.startListening()
    radio.stopListening()

    counts = [0] * CHANNELS
    print(f"Scanning {CHANNELS} channels x {SWEEPS} sweeps... (~10-20 s)\n")

    for _ in range(SWEEPS):
        for ch in range(CHANNELS):
            radio.setChannel(ch)
            radio.startListening()
            time.sleep(0.00013)            # ~130 us dwell for the RPD to settle
            if _rpd(radio):
                counts[ch] += 1
            radio.stopListening()

    radio.powerDown()

    # Histogram: each channel is a capped hex digit (0-f), '.' = no activity.
    print("channel activity ('.' = quiet, f = busy):\n")
    for start in range(0, CHANNELS, 42):
        row = "".join("." if counts[c] == 0 else format(min(counts[c], 15), "x")
                      for c in range(start, min(start + 42, CHANNELS)))
        print(f"  ch {start:3d}-{min(start + 41, CHANNELS - 1):3d}: {row}")

    total = sum(counts)
    print(f"\nTotal detections: {total}")
    if total == 0:
        print(">>> ZERO activity on every channel. The nRF24 is NOT receiving —"
              " module, antenna, or power/brownout (the capacitor). The radio is the bug.")
    else:
        busy = sorted(range(CHANNELS), key=lambda c: counts[c], reverse=True)[:8]
        print(f">>> RX WORKS — busiest channels: {busy}. The radio receives fine, so the"
              " XN297 sniff issue is protocol/alignment, not hardware.")


if __name__ == "__main__":
    main()
