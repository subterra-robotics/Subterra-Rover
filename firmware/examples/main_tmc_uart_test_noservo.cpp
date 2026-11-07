#include <Arduino.h>
#include <TMCStepper.h>
#include <ESP32Servo.h>

// ---- Pins (Schematic_V1) ----
#define STEPPER_UART_TX  25   // module pin 10 -> R1 1k -> driver UART
#define STEPPER_UART_RX  36   // module pin 4 (SENSOR_VP), input-only
#define STEPPER_EN       27   // module pin 12, 10k pull-up -> boots disabled, LOW = enabled
#define STEPPER_STEP     32   // module pin 8
#define STEPPER_DIR      33   // module pin 9
#define STEPPER_DIAG     34   // module pin 6, "StepperMotor_INTERRUPT", input-only

// ---- Driver / motor ----
#define R_SENSE      0.11f    // check the Adafruit TMC2209 board's sense resistor
#define DRIVER_ADDR  0b00     // MS1 = GND, MS2 = GND  -> address 0

const uint16_t MOTOR_CURRENT_MA = 400;   // schematic note: 0.4 A
const uint16_t MICROSTEPS       = 16;
const uint32_t STEPS_PER_REV    = 200UL * MICROSTEPS;
const uint16_t STEP_DELAY_US    = 150;

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDR);

Servo s; 

volatile bool diagFlag = false;
void IRAM_ATTR onDiag() { diagFlag = true; }

void stepMotor(uint32_t steps, bool dir) {
  digitalWrite(STEPPER_DIR, dir);
  delayMicroseconds(5);
  for (uint32_t i = 0; i < steps; i++) {
    digitalWrite(STEPPER_STEP, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEPPER_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void printStatus() {
  driver.DRV_STATUS();
  Serial.printf("  cs_actual=%u stealth=%u standstill=%u\n",
                driver.cs_actual(), driver.stealth(), driver.stst());
  Serial.printf("  ot=%u otpw=%u shortA=%u shortB=%u openA=%u openB=%u\n",
                driver.ot(), driver.otpw(), driver.s2ga(), driver.s2gb(),
                driver.ola(), driver.olb());
}

uint8_t crc8(uint8_t* d, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t b = d[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (b & 1)) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
      b >>= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TMC2209 UART test ===");

  pinMode(STEPPER_EN, OUTPUT);
  digitalWrite(STEPPER_EN, HIGH);          // disabled while configuring
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);
  pinMode(STEPPER_DIAG, INPUT);            // GPIO34 has no internal pull
  attachInterrupt(digitalPinToInterrupt(STEPPER_DIAG), onDiag, RISING);

  s.setPeriodHertz(50); 
  s.attach(13, 500, 2400);
  if (!s.attached()) {
    Serial.println("Servo attach failed");
  }

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
  driver.rms_current(MOTOR_CURRENT_MA);
  driver.microsteps(MICROSTEPS);
  driver.en_spreadCycle(false);   // StealthChop
  driver.pwm_autoscale(true);
  driver.SGTHRS(0);               // stall detect off; DIAG only flags driver errors

  Serial.printf("Readback: %u mA, %u microsteps\n",
                driver.rms_current(), driver.microsteps());

  digitalWrite(STEPPER_EN, LOW);  // enable
}

void loop() {
  Serial.println("1 rev CW");
  stepMotor(STEPS_PER_REV, HIGH);
  Serial.printf("  driver sees DIR=%lu (expect 1)\n", (driver.IOIN() >> 9) & 1);
  delay(500);

  Serial.println("1 rev CCW");
  stepMotor(STEPS_PER_REV, LOW);
  Serial.printf("  driver sees DIR=%lu (expect 0)\n", (driver.IOIN() >> 9) & 1);
  delay(500);

  Serial.println("UART-only spin (VACTUAL) 2 s");
  driver.VACTUAL(4500);           // ~1 rev/s at 16 microsteps
  delay(1000);
  printStatus();
  delay(1000);
  driver.VACTUAL(0);

  Serial.println("At rest:");
  printStatus();

  // (tilt servo sweep 0/90/180 removed: full range can jam the tilt linkage)

  if (diagFlag) {
    Serial.println("!! DIAG pin fired (driver error)");
    diagFlag = false;
  }
  delay(1500);
}