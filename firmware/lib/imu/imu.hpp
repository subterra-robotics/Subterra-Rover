#pragma once
#include <stdint.h>
#include <stdbool.h>

// imu - MPU-6050 driver with a complementary filter for roll / pitch and a
// gyro-integrated relative yaw. Can be polled with imu_read() or kept
// up to date by a background FreeRTOS task (imu_start_task()).
//
// Board mounting on the rover: +X forward, +Y left, +Z up.

#define IMU_OK             0
#define IMU_ERR_NOT_INIT  -1     // imu_init() not called / failed
#define IMU_ERR_BAD_DATA  -2     // read failed or values impossible (bus fault)

// Everything the IMU produces in one sample: filtered orientation,
// calibrated raw values and health/timing info. Filled by imu_read().
struct ImuData {
  // Orientation (complementary filter)
  float roll_deg;          // + = right side down
  float pitch_deg;         // + = nose up (pipe slope)
  float yaw_deg;           // relative heading, gyro only, drifts. Zero with imu_reset_yaw()
  float grade_pct;         // tan(pitch) * 100

  // Calibrated sensor values
  float accel_x_g, accel_y_g, accel_z_g;    // offsets removed
  float accel_mag_g;                        // ~1.0 when still; spikes = bumps / impacts
  float gyro_x_dps, gyro_y_dps, gyro_z_dps; // bias removed
  float temp_c;                             // die temperature

  // Timing / status
  uint32_t timestamp_ms;   // millis() of the last GOOD sample
  float    dt_s;           // time between the last two good samples
  uint32_t sample_count;   // good samples since init
  uint16_t error_count;    // bad reads in a row (0 = fine)
  bool     healthy;        // false -> stop trusting this data (nav should stop)
  bool     calibrated;     // gyro bias has been measured
};

// Probes the MPU-6050 (WHO_AM_I), wakes it and sets +/-2 g, +/-250 dps and a
// 21 Hz digital low-pass. Seeds roll/pitch from the accelerometer.
// Wire.begin() must already have been called. Returns false if not found.
bool imu_init();

// Measures the gyro bias over ~1 s (CAL_SAMPLES). The rover must be still.
void imu_calibrate();

// Measures accelerometer offsets over ~1 s so the current pose reads as
// level. The rover must be still on a LEVEL surface.
void imu_calibrate_level();

// Takes one sample, runs the filter and fills `data`. On a bad read the
// orientation fields keep their last good values and `healthy` drops after
// BAD_READ_LIMIT failures in a row. Returns IMU_OK or an IMU_ERR_* code.
int  imu_read(ImuData &data);

// Sets the relative heading (yaw) back to 0.
void imu_reset_yaw();

// Starts a FreeRTOS task (core 1) that calls imu_read() at rate_hz and
// copies each result into *out under a spinlock. Read it with
// imu_get_snapshot(). Returns false if not initialised or the task failed.
bool imu_start_task(ImuData *out, uint16_t rate_hz);

// Copies *src to *dst atomically with respect to the IMU task, so no field
// is half-updated.
void imu_get_snapshot(const ImuData *src, ImuData *dst);
