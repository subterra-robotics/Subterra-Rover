# Sewer Rover

Autonomous rover that drives itself through newly installed sewer pipe and records video for inspection, using sonar and an IMU to steer and adjust speed on grade changes — no tether, no range limit.

## Overview

### Problem
Pipe is inspected during installation, before it goes into service. The standard tool is a tethered CCTV crawler, which is limited to about 1000 ft of cable. Crews hit that limit partway down a run and have to stop wherever the tether ends rather than wherever makes sense.

Cutting the tether removes the limit. If the rover drives itself, it doesn't need a live control link — only the recorded video matters, and that can be pulled off the rover when it comes out.

### Approach

The rover runs the length of the pipe under its own control and stores video onboard.

- Camera — records the pipe wall for later review
- Sonar — distance to the pipe wall, used to stay centered and detect the end of the run
- IMU / accelerometer — pitch and roll, so the rover knows when it's on an incline or decline
- MCU — reads the sensors and decides steering and speed; slows on declines, adds power on inclines

## Status

Early design. Currently Designing Rover, testing electrical components, and researching Camera and camera MCU.


## Repo structure

```
firmware/       Rover firmware
hardware/       Schematics, PCB layouts, BOM
tools/          Offloading and reviewing recorded video
docs/           Design notes, test logs, protocol spec

```

## Hardware

| Subsystem | Part | Status |
|---|---|---|
| MCU (Rover) | ESP32 | Selected |
| Camera | TBD | Not selected |
| MCU (Camera) | TBD | Not selected |
| Video Storage | TBD | Not selected |
| Sensors | IMU / accelerometer | Possibly more |
| Power | TBD | Not selected |

Update this table as decisions land. Record the reasoning in `docs/`, not here.

## Getting started



## Team

- Yunus Hassen — Computer Engineer -> firmware and electronics
    
- Nael Shakhsheer — Mechanical Engineer -> rover chassis design and 3D printing

- Hannan Ahmed — Cybersecurity Engineer -> camera integration and recording

## Contributing

Work on a branch, open a pull request, get one review before merging. Don't force-push `main`.

## License

TBD