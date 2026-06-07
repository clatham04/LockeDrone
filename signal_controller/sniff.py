"""sniff.py — capture your stock controller to identify the protocol for real.

Puts the nRF24 in RECEIVE mode (XN297/Bayang-aware), listens for the REAL
controller, descrambles + CRC-checks every packet, and prints the valid ones:

  - If we see valid packets -> it IS Bayang, and we get the controller's exact
    address + hop channels (so we can copy it perfectly).
  - If we see nothing either way -> it's likely a different protocol, and we pivot.

HOW TO RUN
    1. Power ON the stock controller (and the drone) within ~1 m of the Pi.
    2. python sniff.py
    3. When it says "Watching channel 0", power-cycle the controller — a BIND packet
       reveals the address AND all four hop channels in one shot.
    4. Paste the WHOLE output back to me.
"""
import time

from pyrf24 import RF24, RF24_1MBPS, RF24_CRC_DISABLED, RF24_PA_MAX

import bayang

CE_PIN = 22
CSN_PIN = 0
PAYLOAD_LEN = bayang.ADDR_LEN + bayang.PACKET_SIZE + 2     # 22 bytes for Bayang


def crc_ok(buf):
    """True if the received 22-byte payload passes the XN297/Bayang CRC."""
    body = buf[:bayang.ADDR_LEN + bayang.PACKET_SIZE]
    crc = bayang._crc16(body)
    crc ^= bayang.XN297_CRC_XOROUT_SCRAMBLED[bayang.ADDR_LEN - 3 + bayang.PACKET_SIZE]
    return ((crc >> 8) & 0xFF) == buf[-2] and (crc & 0xFF) == buf[-1]


def descramble(buf):
    """Undo XN297 scrambling -> (controller_address, 15-byte packet)."""
    addr = bytes(reversed(bytes(buf[i] ^ bayang.XN297_SCRAMBLE[i]
                                for i in range(bayang.ADDR_LEN))))
    packet = bytes(
        bayang._bit_reverse(buf[bayang.ADDR_LEN + i] ^ bayang.XN297_SCRAMBLE[bayang.ADDR_LEN + i])
        for i in range(bayang.PACKET_SIZE)
    )
    return addr, packet


def open_rx(reverse_sync):
    radio = RF24(CE_PIN, CSN_PIN)
    if not radio.begin():
        raise RuntimeError("nRF24 not responding — run test_radio.py first.")
    radio.setDataRate(RF24_1MBPS)
    radio.setCRCLength(RF24_CRC_DISABLED)
    radio.setAutoAck(False)
    radio.setAddressWidth(5)
    radio.setPALevel(RF24_PA_MAX)
    radio.setPayloadSize(PAYLOAD_LEN)
    sync = bayang.XN297_SYNC_ADDRESS
    radio.openReadingPipe(1, bytes(reversed(sync)) if reverse_sync else sync)
    radio.startListening()
    return radio


def listen(radio, channel, seconds):
    radio.setChannel(channel)
    out = []
    end = time.time() + seconds
    while time.time() < end:
        if radio.available():
            buf = bytes(radio.read(PAYLOAD_LEN))
            if len(buf) == PAYLOAD_LEN and crc_ok(buf):
                out.append(descramble(buf))
    return out


def report(channel, addr, packet):
    p0 = packet[0]
    kind = ("BIND" if p0 in (0xA1, 0xA2, 0xA3, 0xA4, 0x53)
            else "CTRL" if p0 in (0xA5, 0xA6) else f"?{p0:#04x}")
    print(f"  ch {channel:3d}  {kind:5s}  addr={addr.hex()}  packet={packet.hex()}")


def sweep(reverse_sync):
    tag = "REVERSED" if reverse_sync else "normal"
    print(f"\n==== sync address: {tag} ====")
    radio = open_rx(reverse_sync)
    hits = 0

    print("Watching channel 0 for 6s — power-cycle the controller NOW...")
    for addr, packet in listen(radio, 0, 6.0):
        report(0, addr, packet)
        hits += 1

    print("Scanning channels 0-83 for control packets...")
    for ch in range(84):
        for addr, packet in listen(radio, ch, 0.15):
            report(ch, addr, packet)
            hits += 1

    radio.powerDown()
    return hits


def main():
    for reverse_sync in (False, True):
        if sweep(reverse_sync):
            print("\n>>> Valid Bayang packets captured above. It IS Bayang — copy the "
                  "addr into TX_ID and we replicate it exactly.")
            return
        print(">>> Nothing valid with this address order.")
    print("\nNo Bayang packets captured either way. Either the controller wasn't "
          "transmitting, was too far, or it's NOT Bayang. Tell me and we'll pivot.")


if __name__ == "__main__":
    main()
