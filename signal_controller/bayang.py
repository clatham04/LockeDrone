"""Bayang protocol over an XN297-emulated nRF24L01+.

Byte-exact port of the DIY-Multiprotocol-TX-Module source
(XN297_EMU.ino + Bayang_nrf24l01.ino). Pure logic, no hardware — easy to read and
trace. drone_link.py feeds these packets to the radio.

Pipeline per packet:
    build_*_packet()  -> 15-byte Bayang packet
    xn297_format()    -> on-air nRF24 payload (scrambled addr + scrambled,
                         bit-reversed body + XN297 CRC)
"""

# ---- XN297 emulation tables (verbatim from XN297_EMU.ino) ----
XN297_SCRAMBLE = bytes([
    0xE3, 0xB1, 0x4B, 0xEA, 0x85, 0xBC, 0xE5, 0x66,
    0x0D, 0xAE, 0x8C, 0x88, 0x12, 0x69, 0xEE, 0x1F,
    0xC7, 0x62, 0x97, 0xD5, 0x0B, 0x79, 0xCA, 0xCC,
    0x1B, 0x5D, 0x19, 0x10, 0x24, 0xD3, 0xDC, 0x3F,
    0x8E, 0xC5, 0x2F, 0xAA, 0x16, 0xF3, 0x95,
])

# scrambled, standard-mode CRC xorout table (indexed by addr_len-3+payload_len)
XN297_CRC_XOROUT_SCRAMBLED = [
    0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C,
    0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x814B,
    0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401,
    0x2138, 0x129F, 0xB3A0, 0x2988, 0x23CA, 0xC0CB,
    0x0C6C, 0xB329, 0xA0A1, 0x0A16, 0xA9D0,
]

# Fixed nRF24 hardware address that emulates the 28-bit XN297 preamble (0xC710F55).
XN297_SYNC_ADDRESS = bytes([0x55, 0x0F, 0x71, 0x0C, 0x00])

ADDR_LEN = 5
PACKET_SIZE = 15
RF_NUM_CHANNELS = 4
PACKET_PERIOD_US = 2000      # Bayang sends a packet every ~2 ms
BIND_COUNT = 1000

# Bayang flag bits — packet[2] and packet[3] (verbatim values)
FLAG_RTH      = 0x01         # packet[2]
FLAG_HEADLESS = 0x02
FLAG_FLIP     = 0x08
FLAG_VIDEO    = 0x10
FLAG_PICTURE  = 0x20
FLAG_INVERTED = 0x80         # packet[3]
FLAG_TAKE_OFF = 0x20         # one-click takeoff / landing
FLAG_EMG_STOP = 0x0C         # emergency motor cut

# Neutral 10-bit stick values (center for AER, idle for throttle)
STICK_CENTER = 0x200         # 512
STICK_MIN = 0x000
STICK_MAX = 0x3FF            # 1023


def _bit_reverse(b):
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4)
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2)
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1)
    return b & 0xFF


def _crc16(data):
    """XN297 CRC16: poly 0x1021, init 0xb5d2, MSB-first."""
    crc = 0xB5D2
    for a in data:
        crc ^= a << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def xn297_format(tx_addr, packet):
    """Wrap a Bayang packet into the on-air nRF24 payload (XN297 scrambled, CRC on).

    Mirrors XN297_WritePayload() for the nRF, addr_len=5, scrambled.
    tx_addr is all-zero during bind, the tx id during control.
    """
    buf = bytearray()

    # address: reversed byte order, each XOR scramble[i]
    for i in range(ADDR_LEN):
        buf.append(tx_addr[ADDR_LEN - 1 - i] ^ XN297_SCRAMBLE[i])

    # payload: bit-reversed, each XOR scramble[addr_len + i]
    for i in range(len(packet)):
        buf.append(_bit_reverse(packet[i]) ^ XN297_SCRAMBLE[ADDR_LEN + i])

    # CRC over the whole scrambled buffer, then the per-length xorout
    crc = _crc16(buf)
    crc ^= XN297_CRC_XOROUT_SCRAMBLED[ADDR_LEN - 3 + len(packet)]   # = index 17 for len 15
    buf.append((crc >> 8) & 0xFF)
    buf.append(crc & 0xFF)
    return bytes(buf)


def hopping_channels(tx_id):
    """The 4 RF hop channels, derived from the tx id (default Bayang sub-protocol)."""
    h1 = (tx_id[3] & 0x1F) + 0x10
    h2 = h1 + 0x20
    h3 = h2 + 0x20
    return [0x00, h1, h2, h3]


def _checksum(packet):
    return sum(packet[:PACKET_SIZE - 1]) & 0xFF


def build_bind_packet(tx_id, hops):
    """15-byte Bayang bind packet — carries our tx id + hop channels to the drone."""
    p = bytearray(PACKET_SIZE)
    p[0] = 0xA4                  # no telemetry, no analog aux
    p[1:6] = bytes(tx_id)        # packet[1..5] = tx id
    p[6:10] = bytes(hops)        # packet[6..9] = hop channels
    p[10] = tx_id[0]
    p[11] = tx_id[1]
    p[12] = tx_id[2]
    p[13] = 0x0A
    p[14] = _checksum(p)
    return bytes(p)


def _encode_channel(val10):
    """10-bit channel -> (hi, lo). Dynamic-trim OFF (constant 0x7C, as headless mode uses).

    The receiver reads the value from (hi & 0x03)<<8 | lo; the 0x7C only sits in the
    trim bits, so this is functionally identical to the stock controller for steering.
    """
    val10 = max(STICK_MIN, min(STICK_MAX, val10))
    return ((val10 >> 8) + 0x7C) & 0xFF, val10 & 0xFF


def build_control_packet(tx_id, aileron, elevator, throttle, rudder,
                         flags2=0x00, flags3=0x00):
    """15-byte Bayang control packet. Channels are 10-bit values (0..1023)."""
    p = bytearray(PACKET_SIZE)
    p[0] = 0xA5
    p[1] = 0xFA                  # normal mode (no analog aux)
    p[2] = flags2               # FLIP / RTH / VIDEO / PICTURE / HEADLESS
    p[3] = flags3               # INVERTED / TAKE_OFF / EMG_STOP
    p[4], p[5]   = _encode_channel(aileron)
    p[6], p[7]   = _encode_channel(elevator)
    p[8], p[9]   = _encode_channel(throttle)
    p[10], p[11] = _encode_channel(rudder)
    p[12] = tx_id[2]
    p[13] = 0x0A
    p[14] = _checksum(p)
    return bytes(p)
