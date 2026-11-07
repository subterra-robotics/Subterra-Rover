#pragma once
#include <stdint.h>
#include <stdbool.h>

// LiDAR - Benewake TFmini-S (UART version) on ESP32 UART2. Non-blocking
// frame parser, validity/health tracking, an optional background task and
// the sensor's configuration commands. Pins and baud live in config.hpp.

#define LIDAR_OK             0     // at least one new valid frame
#define LIDAR_NO_DATA        1     // nothing new yet (normal between frames)
#define LIDAR_WEAK_SIGNAL    2     // new frame, but strength too low/saturated to trust
#define LIDAR_ERR_NOT_INIT  -1

// Everything the LiDAR produces: the latest trusted distance, signal
// quality and health counters. Filled by LiDAR_read().
struct LiDAR {
  // Measurement
  uint16_t distance_cm;      // last VALID reading (sensor range ~10-1200 cm)
  float    distance_m;       // same, in metres
  uint16_t signal_strength;  // return strength; < 100 or 65535 = unreliable
  bool     valid;            // true if the LAST frame can be trusted
  float    temp_c;           // sensor internal temperature

  // Timing / status
  uint32_t timestamp_ms;     // millis() when the last frame arrived
  uint32_t frame_count;      // frames received since init
  uint32_t checksum_errors;  // corrupted frames dropped (wiring/noise indicator)
  uint16_t weak_count;       // weak/invalid frames in a row (0 = fine)
  bool     healthy;          // frames arriving and not stuck on weak signal
};

// Starts UART2 and waits up to 500 ms for one checksum-valid frame.
// Returns false if none arrived; the driver stays initialised so later
// LiDAR_read() calls can still pick the sensor up.
bool LiDAR_init();

// Drains all waiting UART bytes, keeps the newest frame and updates `data`.
// Distance only changes on a valid frame. Returns LIDAR_OK, LIDAR_NO_DATA,
// LIDAR_WEAK_SIGNAL or LIDAR_ERR_NOT_INIT. Never blocks.
int  LiDAR_read(LiDAR &data);

// Starts a FreeRTOS task (core 1) that polls LiDAR_read() at 200 Hz and
// copies each result into *out. Read it with LiDAR_get_snapshot().
bool LiDAR_start_task(LiDAR *out);

// Copies *src to *dst atomically with respect to the LiDAR task.
void LiDAR_get_snapshot(const LiDAR *src, LiDAR *dst);

// Sets the output rate in Hz (default 100). 0 = trigger mode.
void LiDAR_set_frame_rate(uint16_t hz);

// Requests a single frame. Only meaningful with frame rate 0.
void LiDAR_trigger();

// Starts (true) or stops (false) the sensor's data stream.
void LiDAR_enable_output(bool on);

// Writes the current configuration to the sensor's flash. Flash has limited
// write cycles, so call it once after changing settings, not in a loop.
void LiDAR_save_settings();
