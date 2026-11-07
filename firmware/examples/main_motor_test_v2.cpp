// Drive motor test (main ESP32 -> I2C -> ESP32-S3-Zero -> DROK 200206)
// Serial monitor keys:
//   1 = forward  TARGET_MPS (0.125 m/s), steering centred
//   2 = backward TARGET_MPS, steering centred
//   3 = turn left : front wheels steer left,  drive forward
//   4 = turn right: front wheels steer right, drive forward
//   0 = stop (coast)    b = brake
//   r = reset encoders  (use it to measure ENC_COUNTS_PER_WHEEL_REV:
//                        r, turn one wheel exactly once by hand, read enc)
//
// Speed control runs here on the main ESP32 (PI on each wheel, using the
// S3's measured counts/s). Until ENC_COUNTS_PER_WHEEL_REV is set in
// motors.hpp it runs open-loop at OPEN_LOOP_DUTY instead.
//
// WHEELS OFF THE GROUND for the first run.

#include <Arduino.h>
#include "config.hpp"
#include "motors.hpp"
#include <ESP32Servo.h>

#define TARGET_MPS       0.125f   // 1/4 of 0.5 m/s
#define OPEN_LOOP_DUTY   25       // minimum duty that starts the motors (15 stalled)
                                  // used until the encoder is calibrated
#define TURN_STEER_DEG   20       // front wheel angle for keys 3 / 4
#define STEER_LEFT_SIGN  (-1)     // -1: left = below STEER_CENTER_DEG (as in the steering test). Flip if 3 steers right
#define CONTROL_MS       50       // matches the S3's speed window
#define PRINT_MS         250
#define KP               40.0f    // duty per 100% speed error
#define KI               150.0f   // duty per second per 100% speed error

motor left, right;
Servo steer;

static void steer_set(int rel)
{
  if (rel >  STEER_MAX_DEG) rel =  STEER_MAX_DEG;
  if (rel < -STEER_MAX_DEG) rel = -STEER_MAX_DEG;
  steer.write(STEER_CENTER_DEG + rel);
}

static int   dir   = 0;           // +1 forward, -1 backward, 0 stopped
static float duty_i[2] = {0, 0};  // integrator (duty) per wheel
static int   duty[2]   = {0, 0};
static bool  s3_ok = false;

static bool closed_loop(void) { return ENC_COUNTS_PER_WHEEL_REV > 0; }

static float target_cps(void)     // counts per second for TARGET_MPS
{
  return TARGET_MPS / (3.14159265f * (float)WHEEL_DIAMETER) * ENC_COUNTS_PER_WHEEL_REV;
}

static void go(int d)
{
  dir = d;
  for (int i = 0; i < 2; i++) { duty_i[i] = OPEN_LOOP_DUTY; duty[i] = OPEN_LOOP_DUTY; }
  set_motor_direction(&left,  d > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
  set_motor_direction(&right, d > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
  Serial.printf(">> %s %.2f m/s (%s)\n", d > 0 ? "FORWARD" : "BACKWARD", TARGET_MPS,
                closed_loop() ? "closed loop" : "OPEN LOOP - encoder not calibrated");
}

static void halt(bool brake)
{
  dir = 0;
  duty[0] = duty[1] = 0;
  motors_stop(&left, &right, brake);
  Serial.println(brake ? ">> BRAKE" : ">> STOP (coast)");
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Drive motor test ===");
  Serial.printf("1 = forward %.3f m/s, 2 = backward, 3 = left, 4 = right, 0 = stop, b = brake, r = reset encoders\n", TARGET_MPS);

  steer.setPeriodHertz(50);
  steer.attach(SERVO_STEER_PIN, 500, 2400);
  steer_set(0);

  s3_ok = motors_init(&left, &right);
  if (!s3_ok) Serial.println("S3-Zero NOT FOUND at 0x42 - check SDA 21->8, SCL 22->9, GND, S3 power/firmware");
  else        Serial.println("S3-Zero found");

  if (!closed_loop())
    Serial.println("NOTE: ENC_COUNTS_PER_WHEEL_REV = 0 -> open loop. Measure it with 'r' + one wheel turn.");
  else
    Serial.printf("Target %.0f counts/s per wheel\n", target_cps());
}

void loop()
{
  // ---- Keys ----
  while (Serial.available()) {
    char c = Serial.read();
    if (!s3_ok && c != '\n' && c != '\r') { s3_ok = motors_init(&left, &right); if (!s3_ok) Serial.println("S3 still not found"); }
    if      (c == '1') { steer_set(0); go(+1); }
    else if (c == '2') { steer_set(0); go(-1); }
    else if (c == '3') { steer_set( STEER_LEFT_SIGN * TURN_STEER_DEG); Serial.println(">> LEFT");  go(+1); }
    else if (c == '4') { steer_set(-STEER_LEFT_SIGN * TURN_STEER_DEG); Serial.println(">> RIGHT"); go(+1); }
    else if (c == '0') halt(false);
    else if (c == 'b') halt(true);
    else if (c == 'r') { motors_reset_encoders(); Serial.println(">> encoders reset"); }
  }

  // ---- Speed control (also keeps the S3 watchdog fed) ----
  static uint32_t last_ctrl = 0;
  static MotorTelemetry t = {};
  if (millis() - last_ctrl >= CONTROL_MS) {
    float dt = (millis() - last_ctrl) / 1000.0f;
    last_ctrl = millis();

    bool got = motors_read(&t);

    if (dir != 0 && closed_loop() && got) {
      float target = target_cps();
      int32_t meas[2] = { t.speed_l, t.speed_r };
      for (int i = 0; i < 2; i++) {
        float actual = dir * (float)meas[i];               // + when moving the commanded way
        float e = (target - actual) / target;               // fraction of target
        duty_i[i] += KI * e * dt;
        if (duty_i[i] < OPEN_LOOP_DUTY) duty_i[i] = OPEN_LOOP_DUTY;   // never below the start-up minimum
        if (duty_i[i] > MOTOR_DUTY_MAX) duty_i[i] = MOTOR_DUTY_MAX;   // anti-windup
        float d = duty_i[i] + KP * e;
        if (d < OPEN_LOOP_DUTY) d = OPEN_LOOP_DUTY;
        if (d > MOTOR_DUTY_MAX) d = MOTOR_DUTY_MAX;
        duty[i] = (int)lroundf(d);
      }
    }

    left.speed  = (dir != 0) ? duty[0] : 0;
    right.speed = (dir != 0) ? duty[1] : 0;
    if (s3_ok) motors_update(&left, &right);
  }

  // ---- Print ----
  static uint32_t last_print = 0;
  if (millis() - last_print >= PRINT_MS) {
    last_print = millis();
    if (closed_loop()) {
      float k = 3.14159265f * (float)WHEEL_DIAMETER / ENC_COUNTS_PER_WHEEL_REV;
      Serial.printf("%-4s duty L=%3d R=%3d  speed L=%+.2f R=%+.2f m/s  enc L=%ld R=%ld  st=0x%02X%s\n",
                    dir > 0 ? "FWD" : dir < 0 ? "BACK" : "STOP", duty[0], duty[1],
                    t.speed_l * k, t.speed_r * k, (long)t.enc_l, (long)t.enc_r, t.status,
                    (dir && (duty[0] >= MOTOR_DUTY_MAX || duty[1] >= MOTOR_DUTY_MAX)) ? "  AT DUTY LIMIT" : "");
    } else {
      Serial.printf("%-4s duty L=%3d R=%3d  speed L=%+ld R=%+ld counts/s  enc L=%ld R=%ld  st=0x%02X\n",
                    dir > 0 ? "FWD" : dir < 0 ? "BACK" : "STOP", left.speed, right.speed,
                    (long)t.speed_l, (long)t.speed_r, (long)t.enc_l, (long)t.enc_r, t.status);
    }
  }

  delay(2);
}
