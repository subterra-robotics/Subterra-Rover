#pragma once
#include <stdint.h>
#include <stdbool.h>

// motors - rear drive motors, driven directly by the main ESP32 through a
// DROK 200206 dual H-bridge. Both DROK channels share the same three ESP32
// pins (ENA+ENB, IN1+IN3, IN2+IN4), so one `motor` controls both rear wheels.
// Pins live in config.hpp.

#define WHEEL_DIAMETER      0.115   // m
#define MOTOR_DUTY_MAX      102     // 40% of 255: normal operating limit
#define MOTOR_DUTY_CEILING  127     // 50% of 24 V = ~12 V, the motors' rating. Hard cap
#define MOTOR_DUTY_MIN      25      // lowest duty that starts the motors

#define MOTOR_FORWARD    1
#define MOTOR_BACKWARD  -1

// State of the (paired) rear drive motors: pins plus the last commanded
// speed and direction. Fill with motor_init(); change only through the API.
struct motor {
    uint8_t pwm_pin;     // DROK ENA (+ENB)
    uint8_t in_a_pin;    // DROK IN1 (+IN3)
    uint8_t in_b_pin;    // DROK IN2 (+IN4)
    int     speed;       // 0-255, clamped to MOTOR_DUTY_MAX
    int     direction;   // MOTOR_FORWARD or MOTOR_BACKWARD
};

// Assigns the DRIVE_* pins from config.hpp, configures the PWM output and
// leaves the motors stopped (coasting). Returns false if PWM could not attach.
bool motor_init(motor* m);

// Sets the PWM duty (0-255). Values are clamped to MOTOR_DUTY_MAX and never
// exceed MOTOR_DUTY_CEILING. 0 = coast.
void set_motor_speed(motor* m, int speed);

// Sets MOTOR_FORWARD or MOTOR_BACKWARD. Reversing while moving coasts for
// 150 ms first so the gearbox is not slammed into reverse (blocking).
void set_motor_direction(motor* m, int direction);

// Stops the motors: brake = true shorts the windings (both INs high),
// brake = false lets them coast.
void motor_stop(motor* m, bool brake);
