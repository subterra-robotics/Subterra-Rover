# Hardware

KiCad 10 project for the rover electronics.

```
hardware/
  SubterraRoverLib.kicad_sym          Custom symbols (dev boards, modules)
  SubterraRover_Schematics/           Schematic project (no PCB layout yet)
../Schematic V1.pdf                   Exported schematic, 5 Oct 2026
../Sewer_Rover_BOM.pdf                Bill of materials, Rev 2, 28 Sep 2026
```

> The schematic and BOM predate several changes (see [Documentation drift](#documentation-drift)). For current pin assignments, `firmware/include/config.hpp` is the source of truth.

## Selected components

### Compute and storage

| Part | Role | Why |
|---|---|---|
| ESP32 DevKitC (ESP32-WROOM-32E) | Main controller: sensors, tower, drive, steering, mission | On hand, dual core + FreeRTOS, enough UARTs/LEDC channels for everything |
| Raspberry Pi 3B+ | Camera recorder and logger | Hardware H.264 encoder, picamera2 support. Chosen over the Pi Zero 2 W for availability and schedule |
| Arducam IMX708 wide-angle camera | Inspection video | 12 MP, wide field of view for the pipe wall. Records 1280×720 at 5 Mbit/s |
| USB flash drive (exFAT, label `ROVER`) | Video and logs | Pull it out and read it on any laptop. Replaced the planned SD card on the ESP32 |

The ESP32 ↔ Pi link is USB (the DevKit's CP2102) instead of GPIO UART: one cable, and no 3.3 V/5 V logic-level concerns.

### Sensing

| Part | Interface | Role |
|---|---|---|
| Benewake TFmini-S (UART version) | UART2, GPIO16/17, 115200 | Single-point ToF LiDAR, 0.1–12 m. Floor watching and blockage measurement |
| MPU-6050 6-axis IMU | I2C, GPIO21/22, 400 kHz, 0x68 | Pitch (pipe grade), roll (centering), relative yaw |

### LiDAR tower (gimbal)

| Part | Interface | Role |
|---|---|---|
| MG90S metal-gear servo | LEDC PWM, GPIO13 | Tilt axis |
| NEMA 14 stepper (StepperOnline 14HM11-0404S, 0.4 A) | via TMC2209 | Pan axis |
| Adafruit TMC2209 driver | STEP/DIR/EN on GPIO32/33/27, single-wire UART on GPIO25/36, DIAG on GPIO34 | 400 mA RMS, 16 microsteps, StealthChop |

### Drive and steering

| Part | Interface | Role |
|---|---|---|
| 2× 12 V 37 mm gearmotors with encoders (200 RPM, 30:1) | via DROK | Rear wheel drive, 115 mm wheels |
| DROK 200206 dual H-bridge (6.5–27 V, 7 A/ch) | PWM GPIO18, IN GPIO19/23, 10 kHz | Both channels jumpered together, driven directly by the ESP32 |
| 2× MG90S servos | LEDC PWM, GPIO14 (shared) | Front-wheel steering, unpowered front wheels |
| Push button | GPIO26, active LOW, external pull-up | Start/stop run (short), shut down Pi (3 s hold) |

### Power

| Part | Role |
|---|---|
| 24 V battery (bench-tested on a 24 V supply) | Main bus. Sized for demo and testing, not long runtime |
| LM2596 buck module, trimmed to 5.0 V | ESP32 rail (`MCU+5V`) |
| Pololu D24V22F5 (5 V, 2.5 A) | Servo rail |
| SparkFun BabyBuck (5 V) | Pi rail (see faults) |
| 1000 µF / 100 µF / 47 µF bulk caps | Servo rail / TMC2209 motor supply / DevKit 5 V |

Logic pull-ups: 4.7 kΩ on I2C, 10 kΩ on stepper EN (keeps the driver off during boot), 1 kΩ in the TMC2209 single-wire UART TX line, 220 Ω in series with each servo signal.

## Known faults and weaknesses

### Power

- **Pi rail can't supply enough current.** The Pi 3B+ draws well over 1 A at peak with the camera encoding, and the current 5 V regulator for it doesn't keep up. Under-voltage throttling shows in `vcgencmd get_throttled` (`ssh_record.py` reports it). A higher-current regulator is still needed.
- **12 V motors on a 24 V bus.** The motors are only safe because the firmware caps PWM duty at 50%. A firmware bug or a stuck-high PWM pin at boot could put the full 24 V across them.
- **No battery monitoring.** Nothing measures pack voltage, so the rover can't tell it's about to brown out mid-pipe. The schematic's regulator power-good line (GPIO35) isn't used by the firmware.
- **Three separate bucks** (MCU, servos, Pi) add wiring, grounds and failure points.

### LiDAR tower

- **Possible stepper step-angle mismatch.** The BOM part (14HM11-0404S) is a **0.9°** (400 steps/rev) motor, but `tower.cpp` computes `STEPS_PER_DEG` from 200 steps/rev. If that motor is fitted, every pan angle is off by 2× and blockage widths will be wrong. Verify and fix one or the other.
- **Open-loop pan, no homing.** The stepper treats its power-up position as 0°. Idle mode releases the coils, so a bump in idle silently shifts the zero. DIAG is wired, but StallGuard is disabled (`SGTHRS = 0`).
- **Tilt servo is non-linear and offset.** Command 0 is not level; level is at +41 and the curve isn't symmetric, so a measured calibration table is needed. A hobby servo also has backlash and jitter, which turn directly into floor-height error at 0.65 m range.
- **Single-point LiDAR.** One beam can't see the whole cross-section. Blockage width needs a slow pan sweep, and the height/width maths assumes a flat face square to the rover.
- **Weak returns.** Dark, wet or very shiny surfaces give low or saturated signal strength. The firmware treats that as "no return / unsafe", which is safe but can stop the rover in a clean pipe.

### Drive and steering

- **No differential drive.** Both DROK channels share pins, so the rear wheels can't be driven at different speeds. Steering relies entirely on the front servos.
- **Steering servos share one pin.** They can't be trimmed independently and twitch at boot.
- **Encoders unused.** The motors have encoders but only one encoder pin is reserved and nothing reads it, so there's no odometry and video can't be tagged with distance down the pipe.

### Sensing

- **MPU-6050 has no magnetometer**, so yaw drifts with no correction (and a magnetometer would be unreliable in steel pipe anyway). The part is also end-of-life and sensitive to motor vibration.

### Platform

- **ESP32 GPIO is nearly full.** Adding encoders, lighting or battery sensing will need pins that are mostly spoken for.
- **No lighting yet.** The BOM's LED array is unbudgeted, but a sealed pipe is dark.
- **No enclosure or sealing.** Electronics are exposed to dust and moisture.

## Documentation drift

`Schematic V1.pdf` / `.kicad_sch` and the BOM no longer match the build:

| Item | Schematic / BOM | Current build |
|---|---|---|
| LiDAR interface | I2C (GPIO21/22, shared with IMU) | UART2 (GPIO16/17) |
| Camera computer | Pi Zero 2 W over GPIO UART | Pi 3B+ over USB |
| Storage | MicroSD breakout on ESP32 (SPI) | Removed; USB drive on the Pi |
| Drive | Not drawn ("GAP" in BOM) | DROK H-bridge + 2× 12 V gearmotors |
| Motor coprocessor | ESP32-S3-Zero / XIAO RA4M1 planned | Removed; ESP32 drives the DROK directly |
| Battery | 3S Li-ion, 11.1 V | 24 V |
| Pi regulator | — | SparkFun BabyBuck |
| Run button | Not drawn | GPIO26 |
| TMC2209 MS2 | Tied to 3.3 V (UART address 2) | Firmware expects MS1 = MS2 = GND (address 0) — check the board |

The `.kicad_pcb` file is empty: there is no board layout yet.

## Possible improvements

**Power**
- One higher-current 5 V buck (≥ 3 A) for the Pi and camera, or power the Pi from a dedicated converter with its own input filtering.
- A 12 V rail (or a 3S pack) for the drive motors so the duty cap is no longer the only protection, plus a fuse and a hardware kill switch.
- Battery voltage sensing on an ADC pin, with a low-battery return-to-start.

**Tower**
- Fix the step-angle constant, then add homing: an optical or hall end-stop, or enable StallGuard on DIAG.
- Replace the MG90S tilt with a digital servo or a second small stepper with an absolute encoder.
- For full cross-section coverage, consider a 2D scanning LiDAR (e.g. an RPLIDAR-class unit) instead of a gimballed single-point sensor.

**Drive**
- Split the DROK channels onto separate pins for differential drive.
- Read both wheel encoders with the ESP32 PCNT peripheral for odometry, and tag video and blockage logs with distance travelled.
- Give each steering servo its own pin.

**Sensing**
- Upgrade to an IMU with on-chip fusion and lower noise (BNO085, ICM-42688-P).
- Mount the IMU on vibration damping.

**Platform**
- Move from jumper wiring to a carrier PCB or protoboard with keyed connectors (JST/XT60). That would also free GPIO through an I/O expander.
- Add constant-current LED lighting (CRI ≥ 90, 4000–5000 K) sized for the camera.
- Sealed enclosure with a thermal path for the Pi and regulators.
- Update the schematic and BOM to the current build, and lay out the PCB.
