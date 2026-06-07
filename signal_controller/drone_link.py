"""Drone link — drives the nRF24L01+ to speak Bayang to the drone.

This is the connector's hardware seam: connect() / bind() / send_signal() /
disconnect(). Built on bayang.py (byte-exact protocol) + pyrf24 (the radio).

============================ VERIFY ON HARDWARE ============================
The Bayang + XN297 logic is ported exactly from the Multiprotocol source, but two
things can only be confirmed on a real module:

  1. nRF24 address byte-order: RF24's open_tx_pipe() and Multiprotocol's raw
     register write can disagree on order. If the drone won't bind, flip
     XN297_ADDRESS_REVERSED below — that's the #1 thing to try.
  2. Pi timing: Python at ~2 ms/packet has jitter. If bind is flaky, see RF_LINK.md
     (the Arduino + Multiprotocol co-processor option).

A wrong setting just means "no bind" — the drone ignores bad-CRC packets, so this
can't damage anything. Always test with PROPS OFF first.
===========================================================================
"""
import time

from pyrf24 import RF24, RF24_1MBPS, RF24_CRC_DISABLED, RF24_PA_MAX

import bayang

# --- radio wiring (BCM pins) ---
CE_PIN = 22
CSN_PIN = 0          # SPI0 CE0  -> /dev/spidev0.0

# Flip this if the drone won't bind (see note above).
XN297_ADDRESS_REVERSED = False

# Our transmitter id. The drone binds to whatever we send — any stable 5 bytes.
TX_ID = bytes([0xA5, 0xC7, 0x33, 0x1B, 0x42])


def _sync_address():
    addr = bayang.XN297_SYNC_ADDRESS
    return bytes(reversed(addr)) if XN297_ADDRESS_REVERSED else addr


def connect():
    """Bring the radio up in XN297/Bayang mode. Returns the RF24 handle."""
    radio = RF24(CE_PIN, CSN_PIN)
    if not radio.begin():
        raise RuntimeError("nRF24 not responding — run test_radio.py first.")

    radio.setDataRate(RF24_1MBPS)
    radio.setCRCLength(RF24_CRC_DISABLED)          # XN297 CRC lives in the payload
    radio.setAutoAck(False)
    radio.setAddressWidth(5)
    radio.setPALevel(RF24_PA_MAX)
    radio.setPayloadSize(bayang.ADDR_LEN + bayang.PACKET_SIZE + 2)   # 22 bytes on air
    radio.openWritingPipe(_sync_address())          # fixed XN297 sync address
    radio.stopListening()                           # transmit mode
    print("[LINK] Radio configured for Bayang/XN297.")
    return radio


def _send(radio, channel, payload):
    radio.setChannel(channel)
    radio.write(payload)                             # auto-ack off -> no ack wait (one-way)


def bind(radio, tx_id=TX_ID, duration_s=4.0):
    """Stream Bayang bind packets so the drone adopts us as its controller.

    Power the drone (PROPS OFF) right before calling this. Returns the hop channels.
    """
    hops = bayang.hopping_channels(tx_id)
    bind_addr = bytes(5)                            # all-zero address during bind
    payload = bayang.xn297_format(bind_addr, bayang.build_bind_packet(tx_id, hops))

    print(f"[LINK] Binding — tx id {tx_id.hex()}, hops {hops}, channel 0 ...")
    period = bayang.PACKET_PERIOD_US / 1_000_000
    end = time.time() + duration_s
    while time.time() < end:
        _send(radio, 0, payload)                    # bind channel is 0
        time.sleep(period)
    print("[LINK] Bind window done.")
    return hops


def send_signal(radio, tx_id, hops, hop_index,
                aileron, elevator, throttle, rudder, flags2=0x00, flags3=0x00):
    """Send one control packet on the next hop channel. Returns the next hop index."""
    packet = bayang.build_control_packet(tx_id, aileron, elevator, throttle, rudder,
                                         flags2, flags3)
    _send(radio, hops[hop_index], bayang.xn297_format(tx_id, packet))
    return (hop_index + 1) % bayang.RF_NUM_CHANNELS


def disconnect(radio):
    radio.powerDown()
    print("[LINK] Radio powered down.")


# --- helpers for the behavior layer: map normalized inputs to 10-bit channels ---

def stick(value):
    """Centered axis: value in -1.0..+1.0  ->  10-bit, 512 center (roll/pitch/yaw)."""
    return max(bayang.STICK_MIN, min(bayang.STICK_MAX,
                                     int(bayang.STICK_CENTER + value * bayang.STICK_CENTER)))


def throttle(value):
    """Throttle: value in 0.0..1.0  ->  10-bit (0..1023)."""
    return max(bayang.STICK_MIN, min(bayang.STICK_MAX, int(value * bayang.STICK_MAX)))
