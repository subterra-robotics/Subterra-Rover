// MPU-6050 minimum test (raw registers, no library)
// Wiring: VCC -> 3V3, GND -> GND, SDA -> GPIO21, SCL -> GPIO22 (AD0 to GND or open)
//
//  1) I2C scan     : lists every address that answers
//  2) WHO_AM_I     : register 0x75 must read 0x68
//  3) Live data    : accel (g), gyro (deg/s), temperature, 5x a second

#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN   21
#define SCL_PIN   22
#define MPU_ADDR  0x68          // 0x69 if AD0 is tied to 3V3

static bool mpu_ok = false;

static bool write_reg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool read_regs(uint8_t reg, uint8_t *buf, uint8_t n)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// Idle I2C lines must sit HIGH (pull-ups). A line stuck LOW hangs every transfer.
static bool lines_ok(void)
{
  Wire.end();
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, INPUT);
  delay(2);
  int sda = digitalRead(SDA_PIN), scl = digitalRead(SCL_PIN);
  Serial.printf("0) Idle lines: SDA=%d SCL=%d  ", sda, scl);
  if (sda && scl) Serial.println("OK (both pulled high)");
  else {
    if (!sda) Serial.print("SDA STUCK LOW  ");
    if (!scl) Serial.print("SCL STUCK LOW  ");
    Serial.println("-> IMU not powered (its pull-ups go to its VCC), short to GND, or damaged IMU");
  }
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);                 // don't hang for seconds on a dead bus
  return sda && scl;
}

static void scan(void)
{
  Serial.print("1) I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " nothing found -> check VCC/GND, SDA 21, SCL 22, pull-ups");
}

static void check_mpu(void)
{
  uint8_t who = 0;
  if (!read_regs(0x75, &who, 1)) {
    Serial.printf("2) WHO_AM_I: no answer at 0x%02X\n", MPU_ADDR);
    mpu_ok = false;
    return;
  }
  Serial.printf("2) WHO_AM_I = 0x%02X %s\n", who,
                who == 0x68 ? "(OK, MPU-6050)" :
                who == 0x70 || who == 0x71 || who == 0x73 ? "(answers, but this is an MPU-6500/9250 family chip)" :
                "(unexpected - clone or damaged?)");

  // Wake up (powers on in sleep), +/-2 g, +/-250 dps, 44 Hz filter
  mpu_ok = write_reg(0x6B, 0x00) && write_reg(0x1C, 0x00) &&
           write_reg(0x1B, 0x00) && write_reg(0x1A, 0x03);
  if (!mpu_ok) Serial.println("   config write failed");
  delay(100);
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MPU-6050 minimum test ===");
  if (lines_ok()) { scan(); check_mpu(); }
  Serial.println("3) Live data (lay it flat: z ~ +1.00 g, gyro ~ 0):");
}

void loop()
{
  static uint32_t last_retry = 0;
  if (!mpu_ok) {                       // keep retrying so a fixed wire shows up
    if (millis() - last_retry > 2000) {
      last_retry = millis();
      if (lines_ok()) { scan(); check_mpu(); }
    }
    return;
  }

  uint8_t b[14];
  if (!read_regs(0x3B, b, 14)) {
    Serial.println("   read failed (connection lost?)");
    mpu_ok = false;
    return;
  }

  int16_t ax = (b[0] << 8) | b[1],  ay = (b[2] << 8) | b[3],  az = (b[4] << 8) | b[5];
  int16_t t  = (b[6] << 8) | b[7];
  int16_t gx = (b[8] << 8) | b[9],  gy = (b[10] << 8) | b[11], gz = (b[12] << 8) | b[13];

  Serial.printf("acc x=%+5.2f y=%+5.2f z=%+5.2f g | gyro x=%+7.2f y=%+7.2f z=%+7.2f dps | %5.1f C\n",
                ax / 16384.0f, ay / 16384.0f, az / 16384.0f,
                gx / 131.0f,   gy / 131.0f,   gz / 131.0f,
                t / 340.0f + 36.53f);
  delay(200);
}
