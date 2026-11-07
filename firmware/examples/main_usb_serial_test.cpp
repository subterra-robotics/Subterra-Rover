// ESP32 <-> Raspberry Pi USB serial test
// Sends a line every 500 ms and echoes back anything the Pi sends.
// On the Pi the ESP32 shows up as /dev/ttyUSB0 (or see /dev/serial/by-id/).

#include <Arduino.h>
#include "config.hpp"

void setup()
{
  PI_SERIAL.begin(PI_BAUD);
}

void loop()
{
  static uint32_t last = 0;
  static uint32_t count = 0;

  // Send a line to the Pi every 500 ms
  if (millis() - last >= 500) {
    last = millis();
    PI_SERIAL.printf("hello from esp32 #%lu\n", (unsigned long)count++);
  }

  // Echo back whatever the Pi sends
  while (PI_SERIAL.available()) {
    char c = PI_SERIAL.read();
    PI_SERIAL.printf("esp32 got: '%c' (0x%02X)\n", c, (uint8_t)c);
  }
}
