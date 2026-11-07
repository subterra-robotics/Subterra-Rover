// Drive motor test: main ESP32 -> DROK 200206 directly (no coprocessor)
// Both rear motors: forward 2 s, backward 2 s, repeat, at minimum duty.
// (set_motor_direction() coasts 150 ms between directions.)
//
// Wiring (ESP32 -> DROK, both channels jumpered together):
//   GPIO18 -> ENA + ENB     GPIO19 -> IN1 + IN3     GPIO23 -> IN2 + IN4
//   3V3 -> DROK logic +5V pin, GND -> DROK logic GND (common with the 24 V supply)
//
// WHEELS OFF THE GROUND for the first run.

#include <Arduino.h>
#include "config.hpp"
#include "motors.hpp"

#define RUN_MS  2000

motor drive;

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 -> DROK drive test ===");

  if (!motor_init(&drive)) Serial.println("PWM attach failed on GPIO18");
  Serial.printf("duty %d/255 (cap %d), %d ms each way\n", MOTOR_DUTY_MIN, MOTOR_DUTY_CEILING, RUN_MS);

  set_motor_direction(&drive, MOTOR_FORWARD);
  set_motor_speed(&drive, MOTOR_DUTY_MIN);
}

void loop()
{
  Serial.println("FORWARD");
  set_motor_direction(&drive, MOTOR_FORWARD);
  delay(RUN_MS);

  Serial.println("BACKWARD");
  set_motor_direction(&drive, MOTOR_BACKWARD);
  delay(RUN_MS);
}
