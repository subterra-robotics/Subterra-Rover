#include "imu.hpp"
#include "config.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <math.h>

#ifndef IMU_I2C_ADDR
#define IMU_I2C_ADDR      0x68     // AD0 = GND
#endif

// ---- Tuning ----
#define IMU_ALPHA         0.98f    // filter weight on the gyro (0.95 - 0.99)
#define CAL_SAMPLES       500      // ~1 s of samples for calibration
#define BAD_READ_LIMIT    10       // bad reads in a row before healthy = false
#define MAX_DT_S          0.1f     // clamp long gaps so one stall can't spike the angle
#define ACCEL_MIN_G       0.3f     // |a| outside this range = bad read / bus fault
#define ACCEL_MAX_G       3.0f

#define G_MS2             9.80665f
#define RAD2DEG           57.29578f

// ---- Private state ----
static Adafruit_MPU6050 mpu;
static bool     initialized = false;
static bool     gyro_calibrated = false;

static float    gyro_bias[3] = {0, 0, 0};   // deg/s
static float    accel_off[3] = {0, 0, 0};   // g

static float    roll = 0, pitch = 0, yaw = 0;   // filter state, deg
static uint32_t last_us = 0;
static uint32_t samples = 0;
static uint16_t bad_reads = 0;

static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;

// Reads accel (g), gyro (deg/s) and die temperature (C) in one burst.
// No calibration is applied.
static void read_sensor(float a[3], float g[3], float *t)
{
  sensors_event_t ea, eg, et;
  mpu.getEvent(&ea, &eg, &et);

  a[0] = ea.acceleration.x / G_MS2;
  a[1] = ea.acceleration.y / G_MS2;
  a[2] = ea.acceleration.z / G_MS2;
  g[0] = eg.gyro.x * RAD2DEG;
  g[1] = eg.gyro.y * RAD2DEG;
  g[2] = eg.gyro.z * RAD2DEG;
  *t   = et.temperature;
}

// Computes roll and pitch from gravity alone (accurate when still, noisy
// when moving). Axes: pitch is rotation about Y (+ = nose up), roll about X
// (+ = right side down).
static void accel_to_angles(const float a[3], float *r, float *p)
{
  *r = atan2f(a[1], a[2]) * RAD2DEG;
  *p = atan2f(a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * RAD2DEG;
}

// Averages n raw samples (2 ms apart) of accel and gyro. Used for
// calibration, so the rover must be still.
static void average_samples(uint16_t n, float a_avg[3], float g_avg[3])
{
  float a[3], g[3], t;
  float sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};

  for (uint16_t i = 0; i < n; i++) {
    read_sensor(a, g, &t);
    for (int k = 0; k < 3; k++) { sa[k] += a[k]; sg[k] += g[k]; }
    delay(2);
  }
  for (int k = 0; k < 3; k++) { a_avg[k] = sa[k] / n; g_avg[k] = sg[k] / n; }
}

// Probes and configures the MPU-6050, then seeds the filter from the
// accelerometer so the angles don't start at 0.
bool imu_init()
{
  if (!mpu.begin(IMU_I2C_ADDR, &Wire)) {
    initialized = false;
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);                                // let the digital filter settle

  float a[3], g[3], t;
  read_sensor(a, g, &t);
  accel_to_angles(a, &roll, &pitch);
  yaw = 0;

  samples   = 0;
  bad_reads = 0;
  last_us   = micros();
  initialized = true;
  return true;
}

// Stores the averaged gyro reading as the bias to subtract from every sample.
void imu_calibrate()
{
  if (!initialized) return;

  float a_avg[3], g_avg[3];
  average_samples(CAL_SAMPLES, a_avg, g_avg);

  for (int k = 0; k < 3; k++) gyro_bias[k] = g_avg[k];
  gyro_calibrated = true;
  last_us = micros();
}

// Stores accel offsets so the averaged reading becomes the ideal (0, 0, 1 g),
// then re-seeds roll/pitch from the corrected values.
void imu_calibrate_level()
{
  if (!initialized) return;

  float a_avg[3], g_avg[3];
  average_samples(CAL_SAMPLES, a_avg, g_avg);

  accel_off[0] = a_avg[0];
  accel_off[1] = a_avg[1];
  accel_off[2] = a_avg[2] - 1.0f;

  float a[3] = { a_avg[0] - accel_off[0], a_avg[1] - accel_off[1], a_avg[2] - accel_off[2] };
  accel_to_angles(a, &roll, &pitch);
  last_us = micros();
}

// One filter step. Rejects samples whose |a| is outside ACCEL_MIN_G..MAX_G
// (a dead bus usually reads all zeros). Otherwise blends the integrated gyro
// (smooth, drifts) with the accel angles (absolute, noisy) using IMU_ALPHA.
// Yaw has no magnetometer correction and drifts.
int imu_read(ImuData &data)
{
  if (!initialized) {
    data.healthy = false;
    return IMU_ERR_NOT_INIT;
  }

  uint32_t now_us = micros();
  float dt = (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt > MAX_DT_S) dt = MAX_DT_S;

  float a[3], g[3], t;
  read_sensor(a, g, &t);

  for (int k = 0; k < 3; k++) {
    a[k] -= accel_off[k];
    g[k] -= gyro_bias[k];
  }

  float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (mag < ACCEL_MIN_G || mag > ACCEL_MAX_G) {
    if (bad_reads < 0xFFFF) bad_reads++;
    data.error_count = bad_reads;
    data.healthy     = (bad_reads < BAD_READ_LIMIT);
    return IMU_ERR_BAD_DATA;
  }
  bad_reads = 0;

  float acc_roll, acc_pitch;
  accel_to_angles(a, &acc_roll, &acc_pitch);

  roll  = IMU_ALPHA * (roll  + g[0] * dt) + (1.0f - IMU_ALPHA) * acc_roll;
  pitch = IMU_ALPHA * (pitch - g[1] * dt) + (1.0f - IMU_ALPHA) * acc_pitch;   // -gy: nose up = +pitch
  yaw  += g[2] * dt;

  samples++;

  data.roll_deg     = roll;
  data.pitch_deg    = pitch;
  data.yaw_deg      = yaw;
  data.grade_pct    = tanf(pitch / RAD2DEG) * 100.0f;

  data.accel_x_g    = a[0];
  data.accel_y_g    = a[1];
  data.accel_z_g    = a[2];
  data.accel_mag_g  = mag;
  data.gyro_x_dps   = g[0];
  data.gyro_y_dps   = g[1];
  data.gyro_z_dps   = g[2];
  data.temp_c       = t;

  data.timestamp_ms = millis();
  data.dt_s         = dt;
  data.sample_count = samples;
  data.error_count  = 0;
  data.healthy      = true;
  data.calibrated   = gyro_calibrated;

  return IMU_OK;
}

// Zeros the relative heading.
void imu_reset_yaw()
{
  yaw = 0;
}

static ImuData *task_out    = nullptr;
static uint32_t task_period = 10;          // ms

// FreeRTOS task body: reads the IMU at a fixed period (vTaskDelayUntil) and
// publishes the whole struct under the spinlock so readers never see a
// half-updated sample.
static void imu_task(void *arg)
{
  ImuData local = {};
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    imu_read(local);

    portENTER_CRITICAL(&imu_mux);
    *task_out = local;
    portEXIT_CRITICAL(&imu_mux);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(task_period));
  }
}

// Converts rate_hz to a period and launches imu_task on core 1, priority 3.
bool imu_start_task(ImuData *out, uint16_t rate_hz)
{
  if (!initialized || out == nullptr || rate_hz == 0) return false;

  task_out    = out;
  task_period = 1000 / rate_hz;
  if (task_period == 0) task_period = 1;

  return xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 3, NULL, 1) == pdPASS;
}

// Copies the shared struct under the IMU spinlock.
void imu_get_snapshot(const ImuData *src, ImuData *dst)
{
  portENTER_CRITICAL(&imu_mux);
  *dst = *src;
  portEXIT_CRITICAL(&imu_mux);
}
