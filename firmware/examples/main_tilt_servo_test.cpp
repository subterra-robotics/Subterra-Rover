// Tilt servo test (MG90S on SERVO_TILT_PIN)
// Serial monitor keys:
//   1 = tilt "up"   (+STEP_DEG)
//   2 = tilt "down" (-STEP_DEG)
// Starts at 0 = SERVO_TILT_CENTER_DEG. Prints relative angle, raw servo angle
// and pulse width so you can compare with the real angle (protractor / phone level).
//
// The stepper driver is not touched: its EN pull-up keeps it disabled.

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.hpp"

#define STEP_DEG      1        // degrees per key press
#define RAW_MIN       0        // servo library limits
#define RAW_MAX       180
#define PULSE_MIN_US  500      // must match attach() below
#define PULSE_MAX_US  2400

Servo tilt;
static int raw = SERVO_TILT_CENTER_DEG;     // what we send to the servo

static void apply_and_print(void)
{
  tilt.write(raw);
  int rel = raw - SERVO_TILT_CENTER_DEG;
  int us  = PULSE_MIN_US + (long)raw * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  Serial.printf("tilt = %+4d deg   (raw %3d deg, %4d us)\n", rel, raw, us);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Tilt servo test ===");
  Serial.println("1 = up (+), 2 = down (-)");

  tilt.setPeriodHertz(50);
  tilt.attach(SERVO_TILT_PIN, PULSE_MIN_US, PULSE_MAX_US);
  if (!tilt.attached()) Serial.println("Servo attach failed");

  apply_and_print();                         // start at 0 (centre)
}

void loop()
{
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == '1')      raw += STEP_DEG;
  else if (c == '2') raw -= STEP_DEG;
  else return;                               // ignore Enter, spaces, etc.

  if (raw > RAW_MAX) raw = RAW_MAX;
  if (raw < RAW_MIN) raw = RAW_MIN;
  apply_and_print();
}
