###### drone specs ######

### DIY Drone Kit with Brushless Motor & Dual Camera ($70 on Amazon) ###
## 1. Structural & Frame Components
* **Carbon Fiber (CF) Support Rods** *High-strength, ultra-lightweight structural rods that form the rigid framework for the drone arms.*
* **3D-Printed Custom Frame Pieces** *Molded chassis components and snap-fit mounting bosses designed to house the delicate electronic boards and secure the battery.*
* **Assembly Hardware Pack** *A comprehensive assortment of small mounting screws and pins required to lock the frame, arms, and motor mounts together.*

## 2. Power & Propulsion
* **Brushless Motors (x4)** *High-efficiency, long-lasting electromagnets (2 Clockwise, 2 Counter-Clockwise) that provide robust wind resistance and flight power.*
* **Flight Propeller Blades** *Aerodynamically optimized rotor blades designed to generate maximum lift and stable thrust.*
* **Spare Propeller Set** *An extra set of backup flight blades for quick field replacements following accidental crashes.*

## 3. Electronics & Vision Systems
* **Main Flight Controller (FC) Board** *The central processing brain of the quadcopter, featuring integrated gyroscopes and stabilization algorithms to regulate flight.*
* **Optical Flow Sensor Module** *A dedicated downward-facing sensor array that tracks terrain patterns to enable automatic, GPS-free indoor hovering.*
* **Dual Camera System** *A Wi-Fi-enabled dual-lens setup consisting of an adjustable front-facing lens and a secondary vertical downward lens for real-time video transmission.*
* **Orientation LED Strips** *Bright navigational accent lights that help the pilot distinguish the front from the back of the aircraft during flight.*

## 4. Control, Power, & Charging
* **Handheld Remote Controller** *A wireless multi-channel transmitter featuring intuitive joysticks, stunt triggers (360° flips), and one-click takeoff/landing controls.*
* **AA Batteries (For Controller)** *Included power cells to operationalize the remote transmitter immediately out of the box.*
* **Modular Lithium-Polymer (LiPo) Batteries (x2)** *High-capacity, rechargeable drone batteries that swap out easily to deliver up to 30 minutes of cumulative flight time.*
* **Dual USB Charging Cables (x2)** *A set of independent USB chargers allowing both LiPo flight batteries to be charged simultaneously.*

## 5. Documentation & Educational Resources
* **STEM Manual & Assembly Guide** *A step-by-step illustrated assembly booklet interspersed with theory lessons on aerodynamics, aviation history, and quadrotor mechanics.*
* **Digital Educator Materials** *Access links to editable PowerPoint presentation slides designed for classroom teachers or parents running a structured STEM 

# Signal Jammer: #
Radio layer

nRF24L01+ with PA/LNA (~$4–6) — the PA/LNA version has a built-in amplifier, much better range than the bare module. You need two: one for sniffing the original remote, one to permanently mount on the ESP32 setup.
100µF capacitor (~$1) — the nRF24L01+ is notorious for voltage instability; solder one across its VCC/GND pins or it'll behave erratically.

Microcontroller

ESP32 dev board (~$8–12, e.g. ESP32-WROOM-32 DevKit) — acts as the custom RC transmitter. Communicates with the Pi over UART and drives the nRF24L01 over SPI.

Wiring & power

Jumper wires — male-to-female and male-to-male (~$5) — for connecting nRF24L01 to both the Pi (sniffing phase) and ESP32 (transmitter phase).
Small breadboard (~$4) — for the sniffing setup and initial ESP32 wiring before you commit to soldering.
3.3V voltage regulator (e.g. AMS1117-3.3) (~$2) — the nRF24L01 runs on 3.3V max; the Pi's 3.3V pin can power it lightly but the ESP32's onboard 3.3V regulator is cleaner for the final build.

Mounting & final build

Small perfboard or proto-PCB (~$3) — for a clean permanent ESP32 + nRF24L01 assembly once wiring is confirmed.
USB-to-UART adapter (e.g. CP2102 or CH340) (~$5) — for flashing and debugging the ESP32 from your laptop independently of the Pi.
Short USB-A to micro-USB or USB-C cable (~$3) — for powering/flashing the ESP32.

Optional but recommended

Logic level analyzer (e.g. clone Saleae 8-channel) (~$10–15) — lets you inspect the SPI traffic between Pi and nRF24L01 during sniffing, and UART between Pi and ESP32. Saves hours of debugging blind.
Small LiPo or 18650 powerbank — to power the Pi + ESP32 stack untethered during flight tests, if you don't already have one.




# Extra payload weight: #
- 46 grames (1.62 oz) for pi
- 
1. Frame Size Breakdown
5-Inch Frames: A 5-inch drone typically has a payload capacity of roughly 150g to 300g while maintaining good agility. A bare Pi is an easy lift, but space on the top plate can get a bit tight. You will need to think about mounting placement so it doesn't block your GPS or FPV antennas.

6 and 7-Inch Frames: These are generally built for long-range flying or heavier cinematic cameras. Their payload capacity jumps to 400g to 800g+. They have much larger top plates, making it significantly easier to mount the Pi, a dedicated power regulator, and any extra sensors (like a Pi Camera).


