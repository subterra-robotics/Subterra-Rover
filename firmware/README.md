# Firmware

Two programs make up the rover's software:

- **ESP32 (C++, PlatformIO / Arduino on FreeRTOS)** — real-time control and the mission. Everything that moves, senses or decides lives here.
- **Raspberry Pi 3B+ (Python)** — the recorder. It records video and writes logs when the ESP32 tells it to, and it never makes driving decisions.

## Design paradigm

### 1. The ESP32 is in charge, the Pi is a peripheral

The ESP32 owns the mission state machine and every actuator. The Pi only answers commands. If the Pi crashes or the cable comes loose, the ESP32 notices the missing heartbeat and stops the rover. If the ESP32 resets, the Pi saves the open clip and waits for it to come back. Neither side ever needs a person or a wireless link mid-run.

### 2. One module per piece of hardware, all with the same shape

Each library in `lib/` follows the same pattern, so they read and get used the same way:

| Piece | Purpose |
|---|---|
| `xxx_init()` | Configure the hardware, return `false` if it didn't answer |
| A plain data struct (`ImuData`, `LiDAR`, `TowerData`) | Measurement **plus** health: timestamp, counters, `healthy` flag |
| `xxx_read(struct&)` | Non-blocking: take what's available, fill the struct, return an `OK` / `ERR_*` code |
| `xxx_start_task(struct*)` | Optional FreeRTOS task that keeps the struct updated at a fixed rate |
| `xxx_get_snapshot(src, dst)` | Copy the struct under a `portMUX` spinlock so no reader sees half an update |

Each struct carries its own health state, and consumers act on that flag instead of guessing. A LiDAR return that's too weak keeps the last good distance and counts up `weak_count`; an IMU sample with an impossible magnitude is dropped and counts up `error_count`. When either flag goes false, the mission stops the run.

### 3. Single writer per actuator

The **tower task** is the only code that touches the tilt servo and pan stepper. Other code *requests* a mode (`tower_set_mode_floor()`) or a measurement (`tower_measure_blockage()`) by setting a volatile request flag, and the task applies it on its next cycle. This avoids two tasks fighting over a servo, and all motion timing (settling, debouncing) stays in one place.

### 4. Cross-module events are global flags

A few mission-level events are `volatile bool`s declared in `config.hpp` and defined once in `src/flags.cpp`:

| Flag | Set by | Meaning |
|---|---|---|
| `g_recording` | mission (on `REC_ON`) | Pi confirmed it is recording |
| `g_stop_moving` | tower task | Obstacle ahead, drive must stop |
| `g_go_back` | tower task | Blockage measured, return to start |

### 5. All wiring in one file

`include/config.hpp` holds every pin, bus address, baud rate and shared limit. Libraries never hard-code pins, so rewiring is a one-file change.

### 6. Fail safe

- Unknown is treated as unsafe: no LiDAR return in floor mode means `FLOOR_NO_RETURN`, never "clear".
- Floor status is median-filtered and debounced (5 frames) so one bad frame can't stop or start the rover.
- Drive duty is clamped twice in `motors.cpp`: a normal limit (40%) and a hard ceiling (50% of 24 V ≈ the 12 V motor rating) that no caller can exceed.
- Reversing direction coasts for 150 ms first to protect the gearboxes.

## Runtime layout (ESP32)

| Task | Rate | Core / priority | Job |
|---|---|---|---|
| `imu` | 100 Hz | 1 / 3 | Read MPU-6050, complementary filter (α = 0.98) |
| `lidar` | 200 Hz poll (sensor sends 100 Hz) | 1 / 3 | Parse TFmini-S frames |
| `tower` | 100 Hz | 1 / 2 | Apply modes, floor detection, blockage measurement |
| Arduino `loop()` | ~200 Hz | 1 / 1 | Mission state machine, button, Pi link |

### Floor detection

In floor mode the LiDAR points 20° below horizontal and hits the floor about 0.65 m ahead. For each new frame:

1. Split IMU pitch into a slow average (the pipe grade, which the floor follows) and the fast remainder (the rover bouncing). Only the fast part corrects the beam angle.
2. Median-filter the range (5 samples) and add the lens-to-axle offset.
3. Height of the hit point relative to the floor: `h = TOWER_HEIGHT_M − r·sin(beam)`.
4. `h > +4 cm` → obstacle, `h < −4 cm` → dip, invalid frame → no return, else clear. Then debounce.

### Blockage measurement

When an obstacle is confirmed, `tower_measure_blockage()` tilts the beam up until the range jumps (the top edge), computes height, re-aims at mid-height, then pans left and right until the beam slips past each side to get width and sideways offset. It sends `BLOCK,...` to the Pi and sets `g_go_back`. It assumes a flat face square to the rover.

## ESP32 ↔ Pi protocol

USB serial at 115200 baud, one ASCII line per message, `\n` terminated.

| Direction | Message | Meaning |
|---|---|---|
| Pi → ESP32 | `READY` | Heartbeat, once a second. Missing for 2.5 s = Pi lost |
| Pi → ESP32 | `REC_ON` / `REC_OFF` | Recording started / stopped |
| Pi → ESP32 | `REC_FAIL,no_usb` | START refused, no USB drive |
| Pi → ESP32 | `KEY,<c>` | Terminal key forwarded (bench testing only) |
| ESP32 → Pi | `START` / `STOP` | Start / stop a clip |
| ESP32 → Pi | `SHUTDOWN` | Save the clip and power the Pi off (long button press) |
| ESP32 → Pi | `BLOCK,esp_ms,dist_m,height_m,width_m,offset_m,capped` | Appended to `blockages.csv` |
| ESP32 → Pi | `STATE,<name>[,reason]` | State change (logged when a reason is given) |
| ESP32 → Pi | `FAULT,<reason>` / `LOG,<text>` | Logged to `rover_log.txt` |

## Mission state machine

`src/main.cpp`:

```
WAIT_FOR_PI --READY--> READY --button--> STARTING --REC_ON--> RUNNING
                                            |  (3 s no ack: FAULT)   |
                                            v                        v
                                         STOPPED <-- button / Pi lost / sensor fault / blockage done
Long press (3 s) anywhere: stop, send SHUTDOWN, back to WAIT_FOR_PI
```

## Directory layout

```
firmware/
  platformio.ini        Board, framework and library dependencies
  include/config.hpp    Pin map, bus settings, global flag declarations
  src/main.cpp          Mission state machine (the program that gets flashed)
  src/flags.cpp         Global flag storage
  lib/imu/              MPU-6050 + complementary filter
  lib/LiDAR/            TFmini-S UART parser and configuration commands
  lib/motors/           DROK H-bridge rear drive
  lib/tower/            LiDAR tower: modes, floor detection, blockage measurement
  examples/             Bench-test mains, one hardware check each
  rasPiCode/            Pi-side scripts
  wokwi.toml, diagram.json   Wokwi simulator setup (ESP32 + MPU-6050)
```

### Running a bench test

`src/main.cpp` (the mission) is the only main that gets built. To run a bench test, copy it from `examples/` over `src/main.cpp`, and restore the mission afterwards with `git checkout src/main.cpp`. Available tests:

| File | Tests |
|---|---|
| `main_sensor_test.cpp` | IMU + LiDAR readout at 5 Hz |
| `main_floor_detect_test.cpp`, `main_blockage_test.cpp` | Tower floor mode and blockage measurement |
| `main_tilt_lidar_test.cpp`, `main_tilt_servo_test.cpp`, `main_gimbal_sweep.cpp`, `main_imu_pitch_steer_test.cpp` | Tilt calibration and gimbal motion |
| `main_drive_direct_test.cpp`, `main_steer_servo_test.cpp` | Drive motors and steering |
| `main_motor_test_v1.cpp`, `main_motor_test_v2.cpp` | Drive through the old ESP32-S3-Zero coprocessor (no longer in the build) |
| `main_test_run_3tests.cpp` | Scripted test runs with the Pi recording |
| `main_tfmini_min_test.cpp`, `main_mpu6050_min_test.cpp`, `main_tmc_uart_diagnostic.cpp`, `main_tmc_uart_test.cpp`, `main_tmc_uart_test_noservo.cpp` | Library-free wiring checks |
| `main_usb_serial_test.cpp`, `main_record_button_test.cpp` | Pi link |

## Building

```
cd firmware
pio run                 # build
pio run -t upload       # flash
pio device monitor      # 115200 baud
```

Dependencies (installed automatically from `platformio.ini`): ESP32Servo, Adafruit MPU6050, TMCStepper.

**Before power-up:** keep the rover still for about 1 s (gyro calibration), and center the pan stepper by hand with power off, since it has no homing and treats its power-up position as 0°.

## Raspberry Pi side

`rasPiCode/`:

| File | Purpose |
|---|---|
| `rover_pi.py` | The recorder used on the rover. Follows the protocol above |
| `rover_pi.service` | systemd unit to start `rover_pi.py` at boot |
| `record_button.py` | Earlier, simpler recorder: toggles on `BTN` lines (superseded by `rover_pi.py`) |
| `ssh_record.py` | Record from an SSH session and report MB/min, projected size per run, power throttling |
| `camera_test.py` | Record a clip, browse recordings, or watch a live MJPEG stream over Wi-Fi |
| `wifi_test.py` | Hotspot range test: logs signal, ping and loss once a second |

Setup (once, Raspberry Pi OS Bookworm):

```
sudo apt install -y python3-picamera2 python3-serial ffmpeg
sudo usermod -aG dialout $USER            # log out and back in

# USB drive: exFAT, volume label ROVER
sudo mkdir -p /mnt/rover
echo 'LABEL=ROVER /mnt/rover exfat defaults,nofail,uid=1000,gid=1000 0 0' | sudo tee -a /etc/fstab
sudo mount -a

# start at boot
sudo cp rover_pi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rover_pi
journalctl -u rover_pi -f                 # live output
```

**About `rover_pi.service`:** it runs `/home/yunus/dev/rover_pi.py` as user `yunus`, not the copy inside this repo. Either copy the script there (`mkdir -p ~/dev && cp rover_pi.py ~/dev/`), or edit `User=`, `WorkingDirectory=` and `ExecStart=` to point at your clone before installing it. After updating the script, run `sudo systemctl restart rover_pi`.

**`wifi_test.py`** reads the hotspot password from an environment variable instead of the source: `sudo ROVER_WIFI_PASS='yourpassword' python3 wifi_test.py hotspot`.

Clips (`clip_YYYYmmdd_HHMMSS.mp4`, 1280×720, 5 Mbit/s), `rover_log.txt` and `blockages.csv` are written to `/mnt/rover/recordings`. A run is refused if the drive is not mounted.

## Known limitations / next steps

- Drive and steering are not yet wired into the mission (`drive_start/stop/go_back` are stubs).
- `TOWER_MODE_SCAN_WALL` is a placeholder.
- Pan stepper moves are blocking and open loop (no homing); while a blockage is being measured the tower task does not publish floor updates.
- Yaw comes from the gyro only and drifts.
- No odometry: the wheel encoder pin is reserved but unused, so distance down the pipe is not logged with the video.
- The Wokwi simulation (`wokwi.toml`, `diagram.json`) covers only the ESP32 and the IMU.
