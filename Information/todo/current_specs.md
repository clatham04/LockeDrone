# F450 Autonomous Drone — Parts List

| # | Part | Description | Price |
|---|------|-------------|-------|
| 1 | **ShareGoo F450 Frame** | 450mm quadcopter airframe with landing skid gear | Already owned |
| 2 | **Raspberry Pi 4B** | Onboard computer running YOLO human tracking | Already owned |
| 3 | **Wertek 20000mAh Power Bank** | Powers the Pi and night vision camera via USB | Already owned |
| 4 | **Night Vision Camera (Pi CSI)** | IR camera module for Pi 4B, day/night auto-switching | Already owned |
| 5 | **MATEK F405 Wing V2 FC** | Flight controller with ICM42688P IMU and DPS310 barometer, ArduPilot/INAV compatible | $87.99 |
| 6 | **A2212 1000KV Motor Kit (4-pack)** | Includes 4x brushless motors, 4x 30A ESCs, and 1045 CW/CCW propellers | $64.29 |
| 8 | **Zeee 3S 3300mAh 50C LiPo (2-pack)** | 11.1V XT60 battery for motors, ESCs, and FC — 50C discharge rating gives plenty of headroom | $45.89 |
| 10 | **Power Module XT60** *(still needed)* | Steps LiPo voltage to 5V for Matek FC, XT60 input to 6-pin output | ~$10.00 |
| 11 | **FW 10 GPS Module** | tracking and satellite positioning | $18.99 |

**Main Build Total: ~$227.16**

---

## Compatibility Notes

- **Motors + ESCs + Props** — A2212 1000KV motors are the community standard for the F450 450mm frame. 30A ESCs have headroom above the motor's 15A max draw. 1045 props matched to 3S voltage.
- **Flight Controller** — Matek F405 Wing V2 accepts the GPS via UART3 (TX3/RX3) for positioning and I2C (SCL/SDA) for the QMC5883L compass. Motors assigned via SERVO1-4_FUNCTION params in ArduCopter.
- **Telemetry** — YoungRC 915MHz air module connects to Matek F405 UART1 (set SERIAL1_PROTOCOL=2, SERIAL1_BAUD=57). Ground module plugs into Alienware PC via USB. Mission Planner connects at 57600 baud.
- **Battery** — 3S 3300mAh 50C matches A2212 1000KV motor voltage rating. Two packs in parallel via Y-harness doubles capacity to 6600mAh without exceeding the F450 thrust-to-weight limit.

---

# Pixhawk Kit — Optional / Alternative FC Setup

| # | Part | Description | Price |
|---|------|-------------|-------|
| 1 | **Pixhawk PX4 2.4.8 Full Kit** | Includes Pixhawk 2.4.8 FC, NEO-M8N GPS, 3DR 915MHz telemetry, OSD module, PPM module, I2C splitter, and power module | $212.00 |

**Pixhawk Kit Total: $212.00**

> **Note:** This kit overlaps with items 7, 9, and 10 in the main build (GPS, telemetry radios, power module). If using the Pixhawk kit instead of the Matek F405, those three items can be removed from the main build, saving ~$96.99 and dropping the main build total to ~$216.17.

---

## Combined Grand Total

| | |
|---|---|
| **Main Build** | ~$208.17 |
| **Pixhawk Kit** | $212.00 | 