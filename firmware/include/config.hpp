#pragma once

// config.hpp - single source of truth for the rover's pin map, bus settings
// and tuning constants that more than one module needs. Every library under
// lib/ includes this file instead of hard-coding pins. Change wiring here only.

// === LiDAR: Benewake TFmini-S on UART2 ===
#define LIDAR_RX_PIN        16      // ESP32 RX <- TFmini TX (green)
#define LIDAR_TX_PIN        17      // ESP32 TX -> TFmini RX (white)
#define LIDAR_BAUD          115200  // TFmini-S factory default

// === IMU: MPU-6050 on I2C ===
#define IMU_SDA_PIN         21
#define IMU_SCL_PIN         22
#define IMU_I2C_ADDR        0x68    // AD0 = GND

// === Raspberry Pi link: USB (UART0 through the DevKit's CP2102) ===
#define PI_SERIAL           Serial
#define PI_BAUD             115200

// === Pan stepper: TMC2209 driver, single-wire UART config + STEP/DIR ===
#define STEPPER_UART_TX     25
#define STEPPER_UART_RX     36      // input-only pin, fine for RX
#define STEPPER_EN          27      // LOW = enabled, external pull-up keeps it off at boot
#define STEPPER_STEP        32
#define STEPPER_DIR         33
#define STEPPER_DIAG        34

#define DRIVER_ADDR         0b00    // MS1 = GND, MS2 = GND -> UART address 0

#define STEPPER_R_SENSE     0.11f   // ohms, Adafruit TMC2209 breakout
#define STEPPER_MA          400     // RMS current, NEMA 14 rated 0.4 A
#define STEPPER_USTEPS      16

// === Tilt servo (MG90S, LEDC PWM) ===
#define SERVO_TILT_PIN         13
#define SERVO_TILT_CENTER_DEG  90   // raw angle for command 0. NOT level: level = command +41 (see tower.cpp table)
#define SERVO_TILT_MAX_DEG     30   // real LiDAR angle limit, +/- from level

// === Steering servos: both front servos share one PWM pin (220R series each) ===
#define SERVO_STEER_PIN    14       // may twitch at boot
#define STEER_CENTER_DEG   90
#define STEER_MAX_DEG      30       // +/- from center, tune to the linkage

// === Rear drive motors: DROK 200206, both channels jumpered together ===
#define DRIVE_PWM_PIN      18       // -> ENA + ENB
#define DRIVE_IN_A_PIN     19       // -> IN1 + IN3
#define DRIVE_IN_B_PIN     23       // -> IN2 + IN4
#define DRIVE_PWM_HZ       10000    // DROK rated max

// === Wheel encoder (reserved, not used yet) ===
#define ENC_PIN            4

// === Record / run button (active LOW, external pull-up) ===
#define REC_BUTTON_PIN     26

// === Mission flags ===
// Cross-module signals between the tower task and the drive/mission code.
// Declared here, defined exactly once in src/flags.cpp.
extern volatile bool g_recording;    // true = Pi is recording
extern volatile bool g_stop_moving;  // true = obstacle ahead, drive must stop
extern volatile bool g_go_back;      // true = blockage measured, return to start
