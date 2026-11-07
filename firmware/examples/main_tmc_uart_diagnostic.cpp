// TMC2209 UART diagnostic (minimum wiring: VIO, GND, VM, MS1/MS2, PDN_UART)
// Repeats every 3 s, so you can wiggle wires and watch what changes.
//
//  1) Line check : drive TX high/low, read the line on RX (no UART)
//  2) Echo check : send bytes; in single-wire mode RX must hear its own TX
//  3) Addr scan  : ask addresses 0-3 for the version register (0x21)

#include <Arduino.h>
#include <TMCStepper.h>

#define TX_PIN   25     // -> 1k -> PDN_UART
#define RX_PIN   36     // VP, direct to PDN_UART
#define R_SENSE  0.11f

TMC2209Stepper drv0(&Serial1, R_SENSE, 0);
TMC2209Stepper drv1(&Serial1, R_SENSE, 1);
TMC2209Stepper drv2(&Serial1, R_SENSE, 2);
TMC2209Stepper drv3(&Serial1, R_SENSE, 3);
TMC2209Stepper *drv[4] = { &drv0, &drv1, &drv2, &drv3 };

static void line_check(void)
{
  Serial1.end();
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);

  digitalWrite(TX_PIN, HIGH); delay(5);
  int hi = digitalRead(RX_PIN);
  digitalWrite(TX_PIN, LOW);  delay(5);
  int lo = digitalRead(RX_PIN);
  digitalWrite(TX_PIN, HIGH);

  Serial.printf("1) Line : TX high -> RX=%d, TX low -> RX=%d   ", hi, lo);
  if (hi == 1 && lo == 0)      Serial.println("OK (line follows TX)");
  else if (hi == 0 && lo == 0) Serial.println("STUCK LOW: something pulls the line to GND (driver PDN pin damaged, short, or RX on wrong pin)");
  else if (hi == 1 && lo == 1) Serial.println("STUCK HIGH: TX not reaching the line (1k / GPIO25 wiring) or RX on wrong pin");
  else                         Serial.println("INVERTED?? check wiring");
}

static void echo_check(void)
{
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(5);
  while (Serial1.available()) Serial1.read();

  const uint8_t msg[4] = { 0x05, 0x00, 0x06, 0x6F };   // read IOIN from address 0 (sync, addr, reg, CRC)
  Serial1.write(msg, 4);
  Serial1.flush();
  delay(10);

  int n = 0;
  Serial.print("2) Echo : got");
  while (Serial1.available()) { Serial.printf(" %02X", Serial1.read()); n++; }
  if (n == 0)      Serial.println(" nothing -> RX not hearing the line");
  else if (n == 4) Serial.println("  -> own echo only, driver silent (no reply)");
  else if (n > 4)  Serial.println("  -> echo + reply: the DRIVER ANSWERED");
  else             Serial.println("  -> partial echo: noisy / weak line");
}

static void addr_scan(void)
{
  Serial.print("3) Scan :");
  bool any = false;
  for (int a = 0; a < 4; a++) {
    drv[a]->begin();
    uint8_t res = drv[a]->test_connection();     // 0 ok, 1 all ones, 2 all zeros
    if (res == 0) {
      Serial.printf("  addr %d = OK (version 0x%02X)", a, drv[a]->version());
      any = true;
    } else {
      Serial.printf("  addr %d = code %u", a, res);
    }
  }
  Serial.println(any ? "" : "   -> no driver answered");
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TMC2209 UART diagnostic ===");
}

void loop()
{
  line_check();
  echo_check();
  addr_scan();
  Serial.println();
  delay(3000);
}
