#include "motors.hpp"
#include "config.hpp"
#include <Arduino.h>
#include <ESP32Servo.h>      // ESP32PWM shares LEDC channels/timers safely with the servos

#define PWM_BITS 8

static ESP32PWM pwm;

// Pushes the motor's current speed and direction onto the pins. Clamps the
// duty to MOTOR_DUTY_MAX and the absolute MOTOR_DUTY_CEILING; speed 0 coasts.
static void apply(motor* m)
{
  int s = m->speed;
  if (s < 0) s = 0;
  if (s > MOTOR_DUTY_MAX)     s = MOTOR_DUTY_MAX;
  if (s > MOTOR_DUTY_CEILING) s = MOTOR_DUTY_CEILING;

  if (s == 0) {
    pwm.write(0);
    digitalWrite(m->in_a_pin, LOW);
    digitalWrite(m->in_b_pin, LOW);
    return;
  }
  bool fwd = (m->direction >= 0);
  digitalWrite(m->in_a_pin, fwd ? HIGH : LOW);
  digitalWrite(m->in_b_pin, fwd ? LOW : HIGH);
  pwm.write(s);
}

// Binds the DRIVE_* pins, drives both IN pins LOW and attaches the PWM
// output at DRIVE_PWM_HZ with 0 duty. Returns whether PWM attached.
bool motor_init(motor* m)
{
  m->pwm_pin   = DRIVE_PWM_PIN;
  m->in_a_pin  = DRIVE_IN_A_PIN;
  m->in_b_pin  = DRIVE_IN_B_PIN;
  m->speed     = 0;
  m->direction = MOTOR_FORWARD;

  pinMode(m->in_a_pin, OUTPUT);
  pinMode(m->in_b_pin, OUTPUT);
  digitalWrite(m->in_a_pin, LOW);
  digitalWrite(m->in_b_pin, LOW);

  pwm.attachPin(m->pwm_pin, DRIVE_PWM_HZ, PWM_BITS);
  pwm.write(0);
  return pwm.attached();
}

// Stores the requested duty and applies it (clamping happens in apply()).
void set_motor_speed(motor* m, int speed)
{
  m->speed = speed;
  apply(m);
}

// Normalizes the direction to +/-1 and applies it. If the direction flips
// while moving, coasts for 150 ms first, then restores the previous speed.
void set_motor_direction(motor* m, int direction)
{
  int d = (direction >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  if (d != m->direction && m->speed > 0) {
    int keep = m->speed;
    m->speed = 0;  apply(m);
    delay(150);
    m->speed = keep;
  }
  m->direction = d;
  apply(m);
}

// Zeros the speed. Brake drives both INs HIGH with full PWM (shorts the
// windings); otherwise both INs go LOW and the motors coast.
void motor_stop(motor* m, bool brake)
{
  m->speed = 0;
  if (brake) {
    digitalWrite(m->in_a_pin, HIGH);
    digitalWrite(m->in_b_pin, HIGH);
    pwm.write(255);
  } else {
    apply(m);
  }
}
