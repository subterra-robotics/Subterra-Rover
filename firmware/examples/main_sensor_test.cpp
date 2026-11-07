// Sensor test: IMU (MPU-6050, I2C) + LiDAR (TFmini-S, UART2)
// Prints accel, gyro, temperature, angles and distance at 5 Hz.
//
// The stepper driver is not touched: its EN pull-up keeps it disabled.

#include <Arduino.h>
#include <Wire.h>
#include "config.hpp"
#include "imu.hpp"
#include "LiDAR.hpp"

#define PRINT_PERIOD_MS   200     // 5 Hz
#define IMU_RATE_HZ       100

ImuData imu;      // kept updated by the IMU task
LiDAR   lidar;    // kept updated by the LiDAR task

static bool imu_ok   = false;
static bool lidar_ok = false;

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== IMU + LiDAR sensor test ===");

  // ---- IMU ----
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);

  imu_ok = imu_init();
  if (imu_ok) {
    Serial.println("IMU found. Keep still - calibrating gyro...");
    imu_calibrate();
    imu_start_task(&imu, IMU_RATE_HZ);
    Serial.println("IMU ready");
  } else {
    Serial.println("IMU NOT FOUND - check SDA 21 / SCL 22, 3V3, GND, address 0x68");
  }

  // ---- LiDAR ----
  lidar_ok = LiDAR_init();
  if (lidar_ok) {
    Serial.println("LiDAR frames arriving");
  } else {
    Serial.println("LiDAR: no frames yet - check 5V, GND, TX->GPIO16, RX->GPIO17");
  }
  LiDAR_start_task(&lidar);      // start anyway so a late sensor still shows up

  Serial.println();
}

void loop()
{
  static uint32_t last = 0;
  if (millis() - last < PRINT_PERIOD_MS) return;
  last = millis();

  // Consistent copies so every printed field comes from the same sample
  ImuData i;
  LiDAR   l;
  imu_get_snapshot(&imu, &i);
  LiDAR_get_snapshot(&lidar, &l);

  Serial.printf("[%7lu ms]\n", (unsigned long)last);

  if (imu_ok) {
    Serial.printf("  ACC  x=%6.3f  y=%6.3f  z=%6.3f g   |a|=%5.3f g\n",
                  i.accel_x_g, i.accel_y_g, i.accel_z_g, i.accel_mag_g);
    Serial.printf("  GYRO x=%7.2f  y=%7.2f  z=%7.2f dps\n",
                  i.gyro_x_dps, i.gyro_y_dps, i.gyro_z_dps);
    Serial.printf("  ANG  roll=%6.2f  pitch=%6.2f  yaw=%7.2f deg   grade=%5.2f%%\n",
                  i.roll_deg, i.pitch_deg, i.yaw_deg, i.grade_pct);
    Serial.printf("  IMU  temp=%5.1f C  healthy=%d  samples=%lu  errors=%u\n",
                  i.temp_c, i.healthy, (unsigned long)i.sample_count, i.error_count);
  } else {
    Serial.println("  IMU  not connected");
  }

  Serial.printf("  LIDAR dist=%4u cm (%.2f m)  str=%5u  valid=%d  temp=%5.1f C\n",
                l.distance_cm, l.distance_m, l.signal_strength, l.valid, l.temp_c);
  Serial.printf("        healthy=%d  frames=%lu  cksum_err=%lu  weak=%u\n\n",
                l.healthy, (unsigned long)l.frame_count,
                (unsigned long)l.checksum_errors, l.weak_count);
}
