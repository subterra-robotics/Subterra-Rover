# Sewer Rover

Teleoperated rover for inspecting and mapping sewer systems, using surface manholes as fixed nodes in a wireless mesh to relay data and localize the rover between junctions.

## Overview

Sewer pipes block conventional radio, and junction locations away from manholes are often undocumented. This project puts a rover in the pipe and a mesh node at each manhole, so the rover stays in contact with the surface as it moves and the team can map junctions that aren't on any existing drawing.

Two parts:

- **Rover** — drives through the pipe, carries the sensor payload and a radio, streams data back.
- **Manhole nodes** — fixed units dropped at each manhole, relaying rover traffic to the operator and acting as known reference points for localization.

## Repo structure

```
firmware/       Rover firmware
hardware/       Schematics, PCB layouts, BOM
host/           Operator-side tooling
docs/           Design notes, test logs, protocol spec

```

## Hardware

| Subsystem | Part | Status |
|---|---|---|
| MCU (rover) | TBD | Not selected |
| Comms / radio | TBD | Under evaluation |
| Antenna | TBD | Under evaluation |
| Drive | TBD | Not selected |
| Sensors | TBD | Not selected |
| Power | TBD | Not selected |

Update this table as decisions land. Record the reasoning in `docs/`, not here.

## Getting started

```bash
git clone <repo-url>
cd sewer-rover
```

Build and flash instructions go here once the toolchain is fixed.

## Status

Early design. Currently deciding on the comms and antenna setup for the rover.

## Team

- Yunus Hassen — Computer Engineer
    -> Worked on firmware and electrical components 
    
- Nael Shakhsheer — Mechanical Engineer
    -> Worked on rover Physical design and printing

- Hannan Ahmed — Cybersecurity Engineer
    -> Worked on recording camera integration

## Contributing

Work on a branch, open a pull request, get one review before merging. Don't force-push `main`.

## License

TBD