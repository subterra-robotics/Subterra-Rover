// main.cpp - Sub-Terra Rover mission (ESP32 runs the mission, Pi records).
//
// States:  WAIT_FOR_PI -> READY -> STARTING -> RUNNING -> STOPPED -> (button) STARTING ...
//
// Button (REC_BUTTON_PIN, active LOW):
//   short press : READY/STOPPED -> start a run,  RUNNING -> stop the run
//   hold 3 s    : stop any run and tell the Pi to shut down cleanly
//
// Messages over USB (one text line each):
//   Pi -> ESP32 : READY (heartbeat, 1/s), REC_ON, REC_OFF
//   ESP32 -> Pi : START, STOP, SHUTDOWN, BLOCK,..., STATE,<name>[,reason], FAULT,<reason>, LOG,<text>
//
// Rover must be STILL for ~1 s after power-up (IMU gyro calibration).
// Drive is still stubbed: the run ends after a blockage is measured.

#include <Arduino.h>
#include "config.hpp"
#include "tower.hpp"

// ---- Timing ----
#define DEBOUNCE_MS           30
#define LONG_PRESS_MS         3000
#define PI_TIMEOUT_MS         2500    // no READY for this long = Pi lost
#define START_ACK_TIMEOUT_MS  3000    // wait this long for REC_ON after START
#define STATUS_PERIOD_MS      1000

// Mission states, and their names as sent in STATE lines.
enum State { ST_WAIT_FOR_PI, ST_READY, ST_STARTING, ST_RUNNING, ST_STOPPED };
static const char *STATE_NAME[] = { "WAIT_FOR_PI", "READY", "STARTING", "RUNNING", "STOPPED" };

static State    state = ST_WAIT_FOR_PI;
static uint32_t last_ready_ms = 0;
static uint32_t start_req_ms  = 0;
static TowerData tw;

// Drive placeholders: log the intent until the motor/steering code is
// wired into the mission.
static void drive_start()   { PI_SERIAL.println("LOG,drive start (stub)"); }
static void drive_stop()    { PI_SERIAL.println("LOG,drive stop (stub)"); }
static void drive_go_back() { PI_SERIAL.println("LOG,drive go back to start (stub)"); }

// Switches state and reports it to the Pi as STATE,<name>[,reason].
// The Pi only logs state changes that carry a reason.
static void set_state(State s, const char *reason = nullptr)
{
  state = s;
  if (reason) PI_SERIAL.printf("STATE,%s,%s\n", STATE_NAME[s], reason);
  else        PI_SERIAL.printf("STATE,%s\n", STATE_NAME[s]);
}

// True if a READY heartbeat arrived within PI_TIMEOUT_MS.
static bool pi_alive(void)
{
  return last_ready_ms != 0 && (millis() - last_ready_ms) < PI_TIMEOUT_MS;
}

// Asks the Pi to start recording and waits in STARTING for its REC_ON.
// The rover does not move until recording is confirmed.
static void start_run(void)
{
  g_recording = false;
  PI_SERIAL.println("START");
  start_req_ms = millis();
  set_state(ST_STARTING);
}

// Ends a run: stops the drive, parks the tower, clears the mission flags,
// tells the Pi to save the clip and enters STOPPED with the given reason.
static void stop_run(const char *reason)
{
  drive_stop();
  tower_set_mode_idle();
  g_stop_moving = false;
  g_go_back     = false;
  PI_SERIAL.println("STOP");
  set_state(ST_STOPPED, reason);
}

// Handles one complete line from the Pi: READY refreshes the heartbeat
// (and leaves WAIT_FOR_PI), REC_ON / REC_OFF update g_recording.
static void handle_line(const char *line)
{
  if (strcmp(line, "READY") == 0) {
    last_ready_ms = millis();
    if (state == ST_WAIT_FOR_PI) set_state(ST_READY);
  } else if (strcmp(line, "REC_ON") == 0) {
    g_recording = true;
  } else if (strcmp(line, "REC_OFF") == 0) {
    g_recording = false;
  }
}

// Non-blocking line reader for the Pi link. Collects bytes into a 64-byte
// buffer, ignores CR, hands each complete line to handle_line() and drops
// lines that are too long.
static void read_pi(void)
{
  static char    buf[64];
  static uint8_t n = 0;

  while (PI_SERIAL.available()) {
    char c = PI_SERIAL.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[n] = '\0';
      if (n > 0) handle_line(buf);
      n = 0;
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    } else {
      n = 0;
    }
  }
}

// Debounced button reader. Returns 1 for a short press (on release),
// 2 once when the button has been held LONG_PRESS_MS, 0 otherwise.
static int read_button(void)
{
  static bool     stable = HIGH, last_raw = HIGH;
  static uint32_t last_change = 0, press_start = 0;
  static bool     long_fired = false;

  bool raw = digitalRead(REC_BUTTON_PIN);
  if (raw != last_raw) { last_raw = raw; last_change = millis(); }

  if (millis() - last_change >= DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) {
      press_start = millis();
      long_fired  = false;
    } else if (!long_fired) {
      return 1;
    }
  }

  if (stable == LOW && !long_fired && millis() - press_start >= LONG_PRESS_MS) {
    long_fired = true;
    return 2;
  }
  return 0;
}

// Opens the Pi link, configures the button, brings up the tower (IMU,
// LiDAR, servo, stepper; rover must be still), parks it in idle and waits
// for the Pi.
void setup()
{
  PI_SERIAL.begin(PI_BAUD);
  pinMode(REC_BUTTON_PIN, INPUT);              // external pull-up
  delay(300);

  PI_SERIAL.println("LOG,boot - keep still, calibrating");
  if (!tower_init()) PI_SERIAL.println("FAULT,tower_init (check IMU / LiDAR / stepper)");
  tower_set_mode_idle();

  set_state(ST_WAIT_FOR_PI);
}

// Mission state machine, ~200 Hz. Each pass: read the Pi link and button,
// handle a long press (stop + SHUTDOWN), then run the current state:
//   READY/STOPPED : start a run on a short press; drop to WAIT_FOR_PI if the Pi goes quiet
//   STARTING      : wait for REC_ON, then floor mode + drive; fault after 3 s
//   RUNNING       : stop on button, Pi loss or sensor fault; on an obstacle,
//                   stop, measure the blockage, then go back and end the run
// Also sends a STATE line once a second.
void loop()
{
  read_pi();
  int btn = read_button();

  if (btn == 2) {
    if (state == ST_RUNNING || state == ST_STARTING) stop_run("shutdown");
    PI_SERIAL.println("SHUTDOWN");
    last_ready_ms = 0;
    set_state(ST_WAIT_FOR_PI, "shutdown");
    return;
  }

  switch (state) {
    case ST_WAIT_FOR_PI:
      if (btn == 1) PI_SERIAL.println("LOG,button ignored - Pi not ready");
      break;

    case ST_READY:
    case ST_STOPPED:
      if (!pi_alive()) { set_state(ST_WAIT_FOR_PI, "pi_lost"); break; }
      if (btn == 1) start_run();
      break;

    case ST_STARTING:
      if (btn == 1) { stop_run("button"); break; }
      if (g_recording) {
        g_stop_moving = false;
        g_go_back     = false;
        tower_set_mode_floor();
        drive_start();
        set_state(ST_RUNNING);
      } else if (millis() - start_req_ms > START_ACK_TIMEOUT_MS) {
        PI_SERIAL.println("FAULT,no_rec_ack");
        stop_run("no_rec_ack");
      }
      break;

    case ST_RUNNING:
      if (btn == 1)     { stop_run("button");  break; }
      if (!pi_alive())  { stop_run("pi_lost"); break; }

      tower_read(tw);
      if (!tw.lidar.healthy) { PI_SERIAL.println("FAULT,lidar"); stop_run("lidar"); break; }
      if (!tw.imu.healthy)   { PI_SERIAL.println("FAULT,imu");   stop_run("imu");   break; }

      if (g_stop_moving) {
        drive_stop();
        PI_SERIAL.println("LOG,rise detected - measuring blockage");
        tower_measure_blockage();              // blocks ~5-15 s
        read_pi();                             // catch up on heartbeats sent meanwhile
        tower_read(tw);

        if (tw.blockage_state == BLOCK_DONE && g_go_back) {
          drive_go_back();
          stop_run("blockage_measured");
        } else {
          PI_SERIAL.println("FAULT,blockage_measure_failed");
          stop_run("blockage_failed");
        }
      }
      break;
  }

  static uint32_t last_status = 0;
  if (millis() - last_status >= STATUS_PERIOD_MS) {
    last_status = millis();
    PI_SERIAL.printf("STATE,%s\n", STATE_NAME[state]);
  }

  delay(5);
}
