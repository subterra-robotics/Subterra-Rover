// Record button test
// Button on REC_BUTTON_PIN (pulled up, pressed = LOW).
// Each press sends "BTN" to the Pi over USB; the Pi starts or stops recording
// and replies "REC_ON" / "REC_OFF".

#include <Arduino.h>
#include "config.hpp"

#define DEBOUNCE_MS   30

void setup()
{
  PI_SERIAL.begin(PI_BAUD);
  pinMode(REC_BUTTON_PIN, INPUT);    // external pull-up (use INPUT_PULLUP if there isn't one)
}

void loop()
{
  static bool     stable_state = HIGH;   // HIGH = released
  static bool     last_read    = HIGH;
  static uint32_t last_change  = 0;

  bool now = digitalRead(REC_BUTTON_PIN);
  if (now != last_read) {                // pin moved: restart the debounce timer
    last_read   = now;
    last_change = millis();
  }

  if ((millis() - last_change) >= DEBOUNCE_MS && now != stable_state) {
    stable_state = now;
    if (stable_state == LOW) {           // pressed (falling edge)
      PI_SERIAL.println("BTN");
    }
  }

  // Discard the Pi's REC_ON / REC_OFF replies for now
  while (PI_SERIAL.available()) {
    PI_SERIAL.read();
  }
}
