"""test_radio.py — prove the nRF24L01+ is wired up and talking over SPI.

Run this FIRST, before any drone/protocol code. It does NOT touch the drone — it
only checks that the Pi can talk to the radio module. If this fails, fix the wiring
or power before going any further (so you're never debugging two unknowns at once).

    pip install pyrf24
    python test_radio.py

Wiring (Raspberry Pi BCM pins):
    nRF24 CE   -> GPIO22        (physical pin 15)
    nRF24 CSN  -> SPI0 CE0      (physical pin 24, /dev/spidev0.0)
    nRF24 SCK  -> GPIO11/SCLK   (physical pin 23)
    nRF24 MOSI -> GPIO10/MOSI   (physical pin 19)
    nRF24 MISO -> GPIO9/MISO    (physical pin 21)
    nRF24 VCC  -> 3.3V ONLY     (5V destroys the module)
    nRF24 GND  -> GND
    Put a 10-100uF capacitor across the module's VCC/GND pins.

Enable SPI first:  sudo raspi-config -> Interface Options -> SPI -> enable -> reboot
"""
from pyrf24 import RF24

CE_PIN = 22     # BCM GPIO wired to the module's CE
CSN_PIN = 0     # SPI0, CE0  ->  /dev/spidev0.0


def main():
    radio = RF24(CE_PIN, CSN_PIN)

    # 1. Can we even bring the radio up?
    if not radio.begin():
        print("[FAIL] radio.begin() failed — the Pi got no response from the module.")
        print("       Check: 3.3V (NOT 5V), the power cap, MOSI/MISO not swapped,")
        print("       CE/CSN pins, and that SPI is enabled in raspi-config.")
        return

    # 2. SPI read/write sanity check: write a register, read it back.
    radio.channel = 76
    readback = radio.channel
    spi_ok = (readback == 76)

    # 3. The library's own connectivity check.
    chip_ok = radio.is_chip_connected

    print(f"[INFO] begin():            OK")
    print(f"[INFO] chip connected:     {chip_ok}")
    print(f"[INFO] channel write/read: wrote 76, read back {readback}  ->  "
          f"{'OK' if spi_ok else 'MISMATCH'}")
    print()
    radio.print_pretty_details()
    print()

    if chip_ok and spi_ok:
        print("[PASS] nRF24 is wired correctly and talking over SPI.")
        print("       Next milestone: the Bayang bind test (props OFF).")
    else:
        print("[FAIL] SPI link looks wrong even though begin() worked.")
        print("       Most common cause: missing power cap or a marginal 3.3V supply")
        print("       (PA/LNA modules spike current and brown out without it).")


if __name__ == "__main__":
    main()
