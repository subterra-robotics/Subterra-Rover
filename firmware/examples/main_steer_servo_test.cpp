// Steering servo test: both front servos on one PWM pin (SERVO_STEER_PIN)
// Serial monitor keys:
//   1 = steer left  (-STEP_DEG)
//   2 = steer right (+STEP_DEG)
//   0 = centre
// Limited to +/- STEER_MAX_DEG around STEER_CENTER_DEG to protect the linkage.

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.hpp"

#define STEP_DEG      5
#define PULSE_MIN_US  500
#define PULSE_MAX_US  2400

Servo steer;
static int rel = 0;      // degrees from centre

static void apply_and_print(void)
{
  int raw = STEER_CENTER_DEG + rel;
  steer.write(raw);
  int us = PULSE_MIN_US + (long)raw * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  Serial.printf("steer = %+3d deg   (raw %3d deg, %4d us)\n", rel, raw, us);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Steering servo test (both servos, one pin) ===");
  Serial.println("1 = left, 2 = right, 0 = centre");

  steer.setPeriodHertz(50);
  steer.attach(SERVO_STEER_PIN, PULSE_MIN_US, PULSE_MAX_US);
  if (!steer.attached()) Serial.println("Servo attach failed");

  apply_and_print();                       // start centred
}

void loop()
{
  if (!Serial.available()) return;

  char c = Serial.read();
  if      (c == '1') rel -= STEP_DEG;
  else if (c == '2') rel += STEP_DEG;
  else if (c == '0') rel = 0;
  else return;                             // ignore Enter, spaces, etc.

  if (rel >  STEER_MAX_DEG) rel =  STEER_MAX_DEG;
  if (rel < -STEER_MAX_DEG) rel = -STEER_MAX_DEG;
  apply_and_print();
}
