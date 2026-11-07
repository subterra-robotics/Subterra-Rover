// Tilt servo + LiDAR test
// Serial monitor keys:
//   1 = tilt UP 5 deg     2 = tilt DOWN 5 deg     0 = level
// Angles are REAL LiDAR angles from the calibration table (same as tower.cpp):
//   0 = level, positive = looking down. Limited to +/-30 deg.
// Prints the LiDAR range, and when looking down, where the beam hits the
// floor and how high that point is (should be ~0 on flat floor).
//
// Wiring: tilt servo signal -> GPIO13 (servo rail 5 V + common GND)
//         TFmini-S: red 5V, black GND, green -> GPIO16, white -> GPIO17

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>
#include "config.hpp"
#include "LiDAR.hpp"

#define STEP_DEG          5.0f
#define LIMIT_DEG         30.0f
#define AXLE_HEIGHT_M     0.238f   // TOWER_HEIGHT_M
#define LENS_FROM_AXLE_M  0.045f   // LIDAR_FROM_AXIS
#define PRINT_MS          250

// ---- Calibration (copied from tower.cpp) ----
// command = servo angle relative to SERVO_TILT_CENTER_DEG; real = LiDAR angle, negative = down
static const float TILT_CMD[]  = { -30, -20, -10,   0,   9,  20, 30, 41, 50, 60, 70, 80, 90 };
static const float TILT_REAL[] = { -50, -45, -40, -30, -20, -10, -5,  0, 10, 20, 30, 40, 60 };
static const uint8_t TILT_N = sizeof(TILT_CMD) / sizeof(TILT_CMD[0]);

Servo tilt;
LiDAR lidar;
static float deg_down = 0;     // 0 = level
static int   raw = 0;

static float real_to_cmd(float real)
{
  if (real <= TILT_REAL[0])          return TILT_CMD[0];
  if (real >= TILT_REAL[TILT_N - 1]) return TILT_CMD[TILT_N - 1];
  for (uint8_t i = 1; i < TILT_N; i++) {
    if (real <= TILT_REAL[i]) {
      float f = (real - TILT_REAL[i - 1]) / (TILT_REAL[i] - TILT_REAL[i - 1]);
      return TILT_CMD[i - 1] + f * (TILT_CMD[i] - TILT_CMD[i - 1]);
    }
  }
  return TILT_CMD[TILT_N - 1];
}

static void set_tilt(float down)
{
  if (down >  LIMIT_DEG) down =  LIMIT_DEG;
  if (down < -LIMIT_DEG) down = -LIMIT_DEG;
  deg_down = down;
  raw = (int)lroundf(SERVO_TILT_CENTER_DEG + real_to_cmd(-down));
  tilt.write(raw);
  Serial.printf(">> tilt %+.0f deg %s  (raw %d)\n", fabsf(down),
                down > 0 ? "DOWN" : down < 0 ? "UP" : "(level)", raw);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Tilt servo + LiDAR test ===");
  Serial.println("1 = up 5 deg, 2 = down 5 deg, 0 = level");

  tilt.setPeriodHertz(50);
  tilt.attach(SERVO_TILT_PIN, 500, 2400);
  set_tilt(0);

  if (!LiDAR_init()) Serial.println("LiDAR: no frames yet - check wiring (green->16, white->17, 5V, GND)");
  LiDAR_start_task(&lidar);
}

void loop()
{
  while (Serial.available()) {
    char c = Serial.read();
    if      (c == '1') set_tilt(deg_down - STEP_DEG);
    else if (c == '2') set_tilt(deg_down + STEP_DEG);
    else if (c == '0') set_tilt(0);
  }

  static uint32_t last = 0;
  if (millis() - last < PRINT_MS) return;
  last = millis();

  LiDAR l;
  LiDAR_get_snapshot(&lidar, &l);
  if (!l.healthy) { Serial.println("LiDAR not healthy (no frames)"); return; }

  Serial.printf("tilt %+5.1f down  raw %3d | lidar %.3f m  str %5u %s",
                deg_down, raw, l.distance_m, l.signal_strength, l.valid ? "" : "WEAK");

  if (deg_down >= 3.0f && l.valid) {               // beam pointing at the floor
    float s = sinf(deg_down * DEG_TO_RAD), c = cosf(deg_down * DEG_TO_RAD);
    float r = l.distance_m + LENS_FROM_AXLE_M;     // from the tilt axle
    float expect = AXLE_HEIGHT_M / s - LENS_FROM_AXLE_M;
    Serial.printf(" | flat floor %.3f m  ahead %.3f m  height %+.3f m",
                  expect, r * c, AXLE_HEIGHT_M - r * s);
  }
  Serial.println();
}
