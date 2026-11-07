// TFmini-S (UART) minimum test, no library
// Wiring: red -> 5V, black -> GND, green (sensor TX) -> GPIO16, white (sensor RX) -> GPIO17
//
//  0) Idle line : sensor TX idles HIGH; LOW = not powered / wrong wire / swapped
//  1) Raw bytes : should show 59 59 ... (frame headers) streaming at 100 Hz
//  2) Version   : sends "get firmware version" and waits for the reply
//  3) Live data : decoded distance / strength / temperature, 5x a second
// Repeats the checks every few seconds until frames arrive.

#include <Arduino.h>

#define RX_PIN   16      // ESP32 RX <- sensor TX (green)
#define TX_PIN   17      // ESP32 TX -> sensor RX (white)
#define BAUD     115200

static uint8_t  frame[9];
static uint8_t  idx = 0;
static uint32_t frames = 0, bad_cksum = 0, last_frame_ms = 0;
static uint16_t dist = 0, strength = 0;
static float    temp_c = 0;
static bool     streaming = false;

static void idle_line(void)
{
  Serial2.end();
  pinMode(RX_PIN, INPUT);
  delay(5);
  int lvl = digitalRead(RX_PIN);
  Serial.printf("0) Idle line GPIO16 = %d  %s\n", lvl,
                lvl ? "(HIGH - sensor TX present)"
                    : "(LOW - sensor not powered, green not on 16, or green/white swapped)");
  Serial2.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(5);
}

static void raw_bytes(void)
{
  while (Serial2.available()) Serial2.read();
  delay(60);                                   // ~6 frames at 100 Hz
  int n = 0, headers = 0, prev = -1;
  Serial.print("1) Raw bytes:");
  while (Serial2.available() && n < 27) {
    int b = Serial2.read();
    Serial.printf(" %02X", b);
    if (b == 0x59 && prev == 0x59) headers++;
    prev = b;
    n++;
  }
  if (n == 0)        Serial.println("  nothing -> no data (power, green wire, or output disabled)");
  else if (headers)  Serial.println("  -> frame headers seen (59 59): sensor is streaming");
  else               Serial.println("  -> bytes but no 59 59: wrong baud rate, noise, or sensor in another output mode");
}

static void version(void)
{
  const uint8_t cmd[4] = { 0x5A, 0x04, 0x01, 0x5F };   // get firmware version
  while (Serial2.available()) Serial2.read();
  Serial2.write(cmd, 4);

  uint32_t start = millis();
  uint8_t  r[7];
  int      k = 0;
  while (millis() - start < 200 && k < 7) {
    if (!Serial2.available()) continue;
    uint8_t b = Serial2.read();
    if (k == 0 && b != 0x5A) continue;               // skip data frames until the reply
    if (k == 1 && b != 0x07) { k = 0; continue; }
    if (k == 2 && b != 0x01) { k = 0; continue; }
    r[k++] = b;
  }
  if (k == 7) Serial.printf("2) Version: v%u.%u.%u  (sensor RX works: white wire OK)\n", r[5], r[4], r[3]);
  else        Serial.println("2) Version: no reply  (white wire / GPIO17, or sensor not accepting commands)");
}

static void parse(void)
{
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (idx < 2) { if (b == 0x59) frame[idx++] = b; else idx = 0; continue; }
    frame[idx++] = b;
    if (idx < 9) continue;
    idx = 0;

    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += frame[i];
    if (sum != frame[8]) { bad_cksum++; continue; }

    dist     = frame[2] | (frame[3] << 8);
    strength = frame[4] | (frame[5] << 8);
    temp_c   = (frame[6] | (frame[7] << 8)) / 8.0f - 256.0f;
    frames++;
    last_frame_ms = millis();
  }
}

static void run_checks(void)
{
  idle_line();
  raw_bytes();
  version();
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TFmini-S minimum test ===");
  run_checks();
}

void loop()
{
  parse();

  bool alive = last_frame_ms && (millis() - last_frame_ms < 200);
  if (alive && !streaming) { streaming = true; Serial.println("3) Live data:"); }
  if (!alive && streaming) { streaming = false; Serial.println("!! frames stopped"); }

  static uint32_t last = 0;
  if (millis() - last >= (streaming ? 200 : 4000)) {
    last = millis();
    if (streaming) {
      bool ok = strength >= 100 && strength != 65535;
      Serial.printf("dist=%4u cm  strength=%5u  temp=%5.1f C  %s  frames=%lu cksum_err=%lu\n",
                    dist, strength, temp_c, ok ? "valid" : "WEAK/SATURATED",
                    (unsigned long)frames, (unsigned long)bad_cksum);
    } else {
      run_checks();                       // keep diagnosing until it streams
    }
  }
}
