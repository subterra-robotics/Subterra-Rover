# Sub-Terra Rover

An untethered, self-driving rover that runs the length of newly installed large-diameter pipe and records inspection video onboard. Built by Team Sub-Terra Rover for George Mason University's Challenge+X, working with Garney Construction.

## The problem

New pipe is inspected during installation, before it goes into service. The standard tool is a tethered CCTV crawler, and its cable limits a run to roughly 1000 ft. Access points can be around 2000 ft apart, so crews end up stopping wherever the tether runs out instead of where it makes sense.

The original Challenge+X brief was a wireless mesh of nodes at manholes so a rover could be driven remotely underground. After meeting with the client, the team learned there would be no permanent installations to host nodes, and that the live link only existed to drive the rover. If the rover drives itself, all that matters is the recorded video, which can be pulled off when the rover comes out. A Wi-Fi range test (2.4 GHz, Pi hotspot) backed this up: packet loss reached 10% around 10–20 m and 50–70% by about 50 m, far short of a 2000 ft run.

## The approach

The rover drives itself through the pipe and records everything locally:

- **Camera + Raspberry Pi** record H.264 video to a removable USB drive.
- **LiDAR scanning tower** (TFmini-S on a tilt servo and pan stepper) watches the floor ahead for obstacles and dips, and measures a blockage's distance, height and width before turning back.
- **IMU** (MPU-6050) gives pitch and roll, so the rover knows the pipe grade and can separate real obstacles from its own bouncing.
- **ESP32** runs the mission: sensor fusion, obstacle detection, drive and steering, and the run state machine. It commands the Pi to start and stop recording over USB.

Target pipe: new or cleaned, dry, circular steel or ductile iron, 36–54 in diameter. Target speed: 0.5 m/s with a rover of at least 6 lb.

```
          USB serial (text lines)
   ESP32  <---------------------->  Raspberry Pi 3B+
     |  START / STOP / BLOCK / STATE      |  camera -> MP4 on USB drive "ROVER"
     |  <- READY heartbeat, REC_ON/OFF     |  logs, blockages.csv
     |
     +-- LiDAR tower: TFmini-S + tilt servo + pan stepper (TMC2209)
     +-- IMU: MPU-6050
     +-- Drive: DROK H-bridge -> 2x rear DC gearmotors
     +-- Steering: 2x front servos
     +-- Run / record button
```

## Status (Nov 2026)

- Sensor drivers (IMU, LiDAR), the LiDAR tower (floor detection and blockage measurement) and the drive-motor driver are written and bench tested.
- The ESP32 ↔ Pi recording link and the boot-time recorder service work.
- The full mission loop is in `firmware/src/main.cpp`, with the drive calls still stubbed.
- Open hardware items are listed in [`hardware/README.md`](hardware/README.md).

## Repository layout

```
firmware/              ESP32 firmware (PlatformIO) and Raspberry Pi scripts
  include/config.hpp     Pin map and shared constants
  lib/                   Drivers: imu, LiDAR, motors, tower
  src/                   Mission main.cpp + global flags
  examples/              Bench tests (swap one into src/main.cpp to run it)
  rasPiCode/             Pi recorder, systemd service, camera and Wi-Fi tests
hardware/              KiCad schematic and symbol library
docs/                  Test data
Schematic V1.pdf       Exported schematic (Oct 2026, partly out of date)
Sewer_Rover_BOM.pdf    Bill of materials (Rev 2, Sep 2026)
```

- Firmware design and how to build: [`firmware/README.md`](firmware/README.md)
- Components, known faults and improvements: [`hardware/README.md`](hardware/README.md)

## Quick start

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. `cd firmware && pio run -t upload && pio device monitor`
3. On the Pi, follow the setup steps in [`firmware/README.md`](firmware/README.md#raspberry-pi-side).

## Team

- **Yunus Hassen** — Computer Engineering: firmware and electronics (point of contact)
- **Nael Shakhsheer** — Mechanical Engineering: chassis design and 3D printing
- **Hannan Ahmed** — Cybersecurity Engineering: camera integration and recording

## Contributing

Work on a branch, open a pull request, and get one review before merging. Don't force-push `main`.

## License

[MIT](LICENSE)
