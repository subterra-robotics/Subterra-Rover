// Gimbal sweep test: stepper (pan) and servo (tilt) oscillate together
// from -20 deg to +20 deg around center.
//
// No homing yet: the stepper treats its position at power-up as 0 deg.
// With power OFF, turn the gimbal by hand to center before switching on.

#include <Arduino.h>
#include <TMCStepper.h>
#include <ESP32Servo.h>
#include "config.hpp"

// ---- Sweep settings ----
#define SWEEP_DEG          20       // +/- from center
#define DEG_STEP_DELAY_MS  15       // pause per 1 deg -> ~0.6 s per 40 deg sweep
#define END_PAUSE_MS       300      // dwell at each end
#define STEP_PULSE_US      150      // half-period of a STEP pulse

const float STEPS_PER_DEG = (200.0f * STEPPER_USTEPS) / 360.0f;   // 8.89 at 1/16

TMC2209Stepper driver(&Serial1, STEPPER_R_SENSE, DRIVER_ADDR);
Servo tilt;

static int32_t stepper_pos = 0;     // current position in microsteps (0 = center)

volatile bool diagFlag = false;
void IRAM_ATTR onDiag() { diagFlag = true; }

// Move the stepper to an absolute angle (deg from center).
static void stepper_goto_deg(float deg)
{
  int32_t target = lroundf(deg * STEPS_PER_DEG);
  int32_t delta  = target - stepper_pos;
  if (delta == 0) return;

  digitalWrite(STEPPER_DIR, delta > 0 ? HIGH : LOW);
  delayMicroseconds(5);

  int32_t n = delta > 0 ? delta : -delta;
  for (int32_t i = 0; i < n; i++) {
    digitalWrite(STEPPER_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEPPER_STEP, LOW);
    delayMicroseconds(STEP_PULSE_US);
  }
  stepper_pos = target;
}

// Move both axes to the same angle (deg from center).
static void gimbal_set(int deg)
{
  stepper_goto_deg((float)deg);
  tilt.write(SERVO_TILT_CENTER_DEG + deg);
}

// Sweep both axes from 'from' to 'to', 1 deg at a time.
static void sweep(int from, int to)
{
  int dir = (to > from) ? 1 : -1;
  for (int d = from; d != to + dir; d += dir) {
    gimbal_set(d);
    delay(DEG_STEP_DELAY_MS);
  }
}

static void print_status(void)
{
  driver.DRV_STATUS();
  Serial.printf("  cs_actual=%u stealth=%u standstill=%u\n",
                driver.cs_actual(), driver.stealth(), driver.stst());
  Serial.printf("  ot=%u otpw=%u shortA=%u shortB=%u openA=%u openB=%u\n",
                driver.ot(), driver.otpw(), driver.s2ga(), driver.s2gb(),
                driver.ola(), driver.olb());
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Gimbal sweep test (+/-20 deg) ===");

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH);          // disabled while configuring
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);
  pinMode(STEPPER_DIAG, INPUT);            // GPIO34 has no internal pull
  attachInterrupt(digitalPinToInterrupt(STEPPER_DIAG), onDiag, RISING);

  tilt.setPeriodHertz(50);
  tilt.attach(SERVO_TILT_PIN, 500, 2400);
  if (!tilt.attached()) Serial.println("Servo attach failed");
  tilt.write(SERVO_TILT_CENTER_DEG);

  Serial1.begin(115200, SERIAL_8N1, STEPPER_UART_RX, STEPPER_UART_TX);
  driver.begin();

  uint8_t res = driver.test_connection();
  if (res != 0) {
    Serial.printf("UART FAILED (code %u)\n", res);
    Serial.println("1 = all 1s: check RX (GPIO36) / R1 wiring, driver 3V3");
    Serial.println("2 = all 0s: check 24V on VM, address (MS1/MS2), TX/RX");
    while (true) delay(1000);
  }
  Serial.printf("UART OK, version 0x%02X (expect 0x21)\n", driver.version());

  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.I_scale_analog(false);
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(STEPPER_MA);
  driver.microsteps(STEPPER_USTEPS);
  driver.en_spreadCycle(false);            // StealthChop
  driver.pwm_autoscale(true);
  driver.SGTHRS(0);                        // stall detect off

  Serial.printf("Readback: %u mA, %u microsteps, %.2f steps/deg\n",
                driver.rms_current(), driver.microsteps(), STEPS_PER_DEG);

  digitalWrite(STEPPER_EN, LOW);           // enable
  delay(500);                              // let the servo reach center

  sweep(0, -SWEEP_DEG);                    // center -> left end
}

void loop()
{
  Serial.println("-> +20");
  sweep(-SWEEP_DEG, SWEEP_DEG);
  delay(END_PAUSE_MS);

  Serial.println("-> -20");
  sweep(SWEEP_DEG, -SWEEP_DEG);
  delay(END_PAUSE_MS);

  if (diagFlag) {
    Serial.println("!! DIAG pin fired (driver error)");
    print_status();
    diagFlag = false;
  }
}
