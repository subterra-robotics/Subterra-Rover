// IMU -> steering test
// Key 1 (serial monitor): set the current roll / pitch / yaw as 0 deg.
// After zeroing: if pitch goes past +/- TILT_LIMIT_DEG, the steering servos
// turn the opposite way. Back inside TILT_RELEASE_DEG they re-centre.
//
// Rover must be STILL for ~1 s at boot while the gyro calibrates.

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "config.hpp"
#include "imu.hpp"

#define TILT_LIMIT_DEG    20.0f   // react past this
#define TILT_RELEASE_DEG  15.0f   // re-centre below this (hysteresis, no chatter)
#define STEER_TURN_DEG    20      // how far to steer when reacting
#define STEER_SIGN        (+1)    // +1: pitch + -> steer +, pitch - -> steer -. Flip to -1 to reverse
#define PRINT_PERIOD_MS   200
#define PULSE_MIN_US      500
#define PULSE_MAX_US      2400

ImuData imu;
Servo   steer;

static bool  zeroed = false;
static float roll0 = 0, pitch0 = 0, yaw0 = 0;
static int   steer_rel = 0;              // current steering, deg from centre

static void steer_set(int rel)
{
  if (rel >  STEER_MAX_DEG) rel =  STEER_MAX_DEG;
  if (rel < -STEER_MAX_DEG) rel = -STEER_MAX_DEG;
  steer_rel = rel;
  steer.write(STEER_CENTER_DEG + rel);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== IMU -> steering test ===");

  steer.setPeriodHertz(50);
  steer.attach(SERVO_STEER_PIN, PULSE_MIN_US, PULSE_MAX_US);
  steer_set(0);

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);
  while (!imu_init()) {
    Serial.println("IMU not found - check wiring");
    delay(1000);
  }
  Serial.println("Keep still - calibrating gyro...");
  imu_calibrate();
  imu_start_task(&imu, 100);

  Serial.println("Ready. Press 1 to set the current position as 0 deg.");
}

void loop()
{
  ImuData i;
  imu_get_snapshot(&imu, &i);

  // ---- Key 1: zero all tilt axes at the current reading ----
  while (Serial.available()) {
    if (Serial.read() == '1') {
      roll0  = i.roll_deg;
      pitch0 = i.pitch_deg;
      yaw0   = i.yaw_deg;
      zeroed = true;
      steer_set(0);
      Serial.printf(">> Zeroed at roll=%.2f pitch=%.2f yaw=%.2f\n", roll0, pitch0, yaw0);
    }
  }

  float roll  = i.roll_deg  - roll0;
  float pitch = i.pitch_deg - pitch0;
  float yaw   = i.yaw_deg   - yaw0;

  // ---- Steering reaction (pitch only) ----
  if (zeroed && i.healthy) {
    if (pitch > TILT_LIMIT_DEG) {
      steer_set(STEER_SIGN * STEER_TURN_DEG);
    } else if (pitch < -TILT_LIMIT_DEG) {
      steer_set(-STEER_SIGN * STEER_TURN_DEG);
    } else if (fabsf(pitch) < TILT_RELEASE_DEG) {
      steer_set(0);
    }                                      // between 15 and 20: keep what it's doing
  } else if (!i.healthy) {
    steer_set(0);                          // IMU lost -> straighten up
  }

  // ---- Print ----
  static uint32_t last = 0;
  if (millis() - last >= PRINT_PERIOD_MS) {
    last = millis();
    Serial.printf("roll=%+7.2f  pitch=%+7.2f  yaw=%+7.2f  steer=%+3d  %s%s\n",
                  roll, pitch, yaw, steer_rel,
                  zeroed ? "" : "(not zeroed) ",
                  i.healthy ? "" : "IMU UNHEALTHY");
  }

  delay(10);
}
