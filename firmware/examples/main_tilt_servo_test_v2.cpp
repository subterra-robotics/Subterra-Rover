// main.cpp - BENCH TEST: tilt servo calibration (MG90S on SERVO_TILT_PIN).
//
// src/main.cpp is swapped between bench tests while hardware is brought up.
// The full mission loop is archived in saveCode/main_mission_v1.cpp.
//
// Serial monitor keys:
//   1 = tilt "up"   (+STEP_DEG)
//   2 = tilt "down" (-STEP_DEG)
// Starts at 0 = SERVO_TILT_CENTER_DEG and prints relative angle, raw servo
// angle and pulse width, so the command can be compared with the real angle
// (protractor / phone level). The stepper stays disabled by its EN pull-up.

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.hpp"

#define STEP_DEG      1        // degrees per key press
#define RAW_MIN       0        // servo library limits
#define RAW_MAX       180
#define PULSE_MIN_US  500      // must match attach() below
#define PULSE_MAX_US  2400

Servo tilt;
static int raw = SERVO_TILT_CENTER_DEG;

// Writes the current raw angle to the servo and prints the relative angle,
// raw angle and the pulse width that angle maps to.
static void apply_and_print(void)
{
  tilt.write(raw);
  int rel = raw - SERVO_TILT_CENTER_DEG;
  int us  = PULSE_MIN_US + (long)raw * (PULSE_MAX_US - PULSE_MIN_US) / 180;
  Serial.printf("tilt = %+4d deg   (raw %3d deg, %4d us)\n", rel, raw, us);
}

// Opens the serial monitor, attaches the tilt servo at 50 Hz and moves it
// to the center position.
void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Tilt servo test ===");
  Serial.println("1 = up (+), 2 = down (-)");

  tilt.setPeriodHertz(50);
  tilt.attach(SERVO_TILT_PIN, PULSE_MIN_US, PULSE_MAX_US);
  if (!tilt.attached()) Serial.println("Servo attach failed");

  apply_and_print();
}

// Steps the servo one STEP_DEG per '1' / '2' key, clamped to the servo's
// raw range. Any other key (Enter, spaces) is ignored.
void loop()
{
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == '1')      raw += STEP_DEG;
  else if (c == '2') raw -= STEP_DEG;
  else return;

  if (raw > RAW_MAX) raw = RAW_MAX;
  if (raw < RAW_MIN) raw = RAW_MIN;
  apply_and_print();
}
