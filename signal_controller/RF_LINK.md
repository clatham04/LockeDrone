# RF Link — Emulating the Controller with an nRF24 on the Pi

How the Raspberry Pi takes the place of the stock 2.4 GHz controller. This is the
real implementation behind `drone_link.py` in [ARCHITECTURE.md](ARCHITECTURE.md):
the connector emits a `ControlSignal`, and this layer turns it into the same RF
packets the drone's receiver already expects.

The drone is never modified. The Pi just becomes a transmitter that speaks the
drone's language.

---

## The plan in one picture

```
  control.py ──ControlSignal──▶ drone_link.py ──SPI──▶ nRF24L01+ ──2.4GHz──▶ drone RX
  (hover/follow)                (Bayang protocol)        (radio)              (stock)
```

---

## How binding works (important — and good news)

Toy drones like this **bind to whoever sends the correct bind packets at power-up.**
They do *not* remember a unique controller forever. So:

1. We make the Pi generate its **own** transmitter ID.
2. On power-up, the Pi streams **bind packets** for the right protocol.
3. The drone accepts the Pi as its controller.
4. The Pi then streams **control packets** ~every few ms.

➡️ We do **not** need to extract anything secret from your controller. We only need
to know **which protocol** it uses. Everything else we generate ourselves.

---

## Step 0 — Identify the protocol (the only real blocker)

99% likely it's **Bayang** (the default for brushless STEM kits with flip / headless
/ one-click takeoff). To confirm, do any one of these:

- **FCC ID:** sticker on the controller or drone, format `2AXXX-XXXX`. Look it up at
  fcc.io / fccid.io — the test report lists the exact frequency + modulation.
- **Chip photo:** open the controller, photograph the chip near the antenna. Markings
  like `BK2421`, `XN297`, `BK2425`, or `nRF24L01` confirm the radio family
  (all Bayang-compatible).
- **Trial bind:** just try to bind with Bayang first (Step 4). If it binds, done. If
  not, we try the next candidates (Bayang variants, then others).

---

## Step 1 — Hardware (~$5)

| Part | Notes |
|------|-------|
| **nRF24L01+ PA/LNA** module (with external antenna) | The PA/LNA version for range. Must be the **+** variant. |
| **10–100 µF capacitor** | Soldered across the module's **VCC↔GND**, right at the pins. These modules are noisy and **will not transmit reliably without it.** |
| Female-female jumper wires | 7 wires (SPI + power). |
| *(Recommended)* nRF24 **adapter/breakout** with onboard 3.3 V regulator | Cleanest power. The Pi's 3.3 V rail alone is marginal for PA/LNA current spikes. |

⚠️ **The nRF24 is 3.3 V only.** 5 V on VCC destroys it. Logic pins are 3.3 V — fine
directly to the Pi.

---

## Step 2 — Wiring (nRF24 ↔ Raspberry Pi, SPI0)

| nRF24 pin | Pi signal | Pi physical pin |
|-----------|-----------|-----------------|
| GND  | GND        | 6  |
| VCC  | 3.3 V      | 1  (or external 3.3 V regulator) |
| CE   | GPIO22     | 15 |
| CSN  | GPIO8 / CE0| 24 |
| SCK  | GPIO11/SCLK| 23 |
| MOSI | GPIO10/MOSI| 19 |
| MISO | GPIO9/MISO | 21 |
| IRQ  | — (unused) | leave unconnected |

Enable SPI on the Pi: `sudo raspi-config` → Interface Options → SPI → enable, reboot.

---

## Step 3 — Software stack

- **`pyrf24`** — maintained Python bindings for the TMRh20 RF24 library:
  `pip install pyrf24`. Gives low-level nRF24 control over SPI.
- **Bayang protocol port** — Bayang chips are XN297-class. The nRF24L01+ can't do
  XN297 natively, so we use the well-known **"XN297 emulation"**: disable hardware
  CRC/auto-ack, prepend the XN297 preamble, and scramble the payload with the XN297
  table. The packet format + scramble table are documented in the
  [Multiprotocol](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module)
  `Bayang.ino` and `goebish/nrf24_multipro` sources — we port that into
  `drone_link.py`.

### Honest engineering note: timing

Bayang hops RF channels every few milliseconds. A microcontroller nails this; the
Pi's non-realtime Linux scheduler makes precise hopping harder (it works, but jitter
can hurt range/reliability). Two options, **same radio either way**:

- **Pi-direct (what you chose):** nRF24 straight on the Pi's SPI. Simplest wiring,
  best for bring-up and bench testing. Start here.
- **MCU co-processor (reliability upgrade):** a tiny ~$3 Arduino/STM32 running
  Multiprotocol firmware does the real-time RF; the Pi just sends 4 channel values
  to it over serial/USB. Light enough to fly. This is the robust version of the
  exact same idea — keep it in your back pocket if Pi-direct timing gets flaky.

---

## Step 4 — Bring-up checklist (PROPS OFF / motors disabled until the end)

1. Wire the module; enable SPI; confirm the Pi can read the nRF24's registers
   (`pyrf24` `print_details()`), proving SPI works.
2. Implement Bayang **bind** + **control** packets in `drone_link.py`.
3. Power the drone with **props removed**. Run the Pi bind sequence. Watch for the
   drone's bind confirmation (LED stops blinking / goes solid).
4. Send a tiny throttle/channel change; confirm motors twitch correctly.
5. Map all channels + buttons (Step 5).
6. Only then, props on, low test in a safe/enclosed space.

---

## Step 5 — Mapping `ControlSignal` → drone channels

The connector's `ControlSignal` (vx, vy, vz, yaw_rate) maps onto the controller's
channels; buttons become packet flags:

| Drone channel | Source |
|---------------|--------|
| Throttle (vertical) | from `vz` / altitude hold |
| Aileron / Roll (`vy`) | sideways velocity |
| Elevator / Pitch (`vx`) | forward velocity |
| Rudder / Yaw (`yaw_rate`) | turn rate |
| Flags byte | one-click takeoff, land, flip, headless, speed — set as needed |

`drone_link.send_signal()` scales each `ControlSignal` field into the Bayang
channel range (typically ~0–1000 / 12-bit), packs the 16-byte packet, XN297-scrambles
it, and transmits on the current hop channel. `read_state()` is trickier — most toy
Bayang links are **one-way** (no telemetry back), so altitude/drift for hover will
come from the drone's **own optical-flow hold** plus whatever the Pi senses
(camera / an add-on rangefinder), **not** from the RC link. We'll account for that
when we wire up the hover behavior.

---

## Open items (need your input / hardware)

- [ ] Confirm protocol: FCC ID **or** chip photo **or** trial bind.
- [ ] Buy the nRF24L01+ PA/LNA module + cap.
- [ ] Note: this kit's RC link is almost certainly **one-way** — flag if your
      controller shows any drone telemetry (battery %, etc.), which would change that.
