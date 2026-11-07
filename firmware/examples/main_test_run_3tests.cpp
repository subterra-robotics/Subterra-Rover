// Sub-Terra Rover - test run (ESP32 runs the mission, Pi records to the USB drive)
//
// Test 1: Idle = LiDAR looking straight forward. Button press -> Pi starts
//         recording, tower goes to floor mode (20 deg down). When a rise is
//         found -> stop recording, back to idle (looking forward).
// Test 2: Simulated tilt from keys typed into rover_pi.py (forwarded as KEY,x):
//           1 = tilt left 5 deg, 2 = tilt right 5 deg, 0 = level
//         Past +/-20 deg the steering servos turn the other way; back inside
//         +/-15 deg they re-centre. Works in every state once the Pi is connected.
// Test 3: The Pi records only to the USB drive; no drive -> REC_FAIL, no run.
//
// Frame: X+ forward, Y+ left, Z+ up.  Roll + = right side down (tilt right).
//
// Button (REC_BUTTON_PIN): short press = start / stop run, hold 3 s = Pi shutdown.
// Rover must be STILL for ~1 s after power-up (IMU gyro calibration).

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.hpp"
#include "tower.hpp"

// ---- Timing ----
#define DEBOUNCE_MS           30
#define LONG_PRESS_MS         3000
#define PI_TIMEOUT_MS         2500    // no READY for this long = Pi lost
#define START_ACK_TIMEOUT_MS  3000    // wait this long for REC_ON after START
#define STATUS_PERIOD_MS      1000

// ---- Simulated tilt -> steering ----
#define SIM_STEP_DEG          5.0f
#define TILT_LIMIT_DEG        20.0f   // react past this
#define TILT_RELEASE_DEG      15.0f   // re-centre inside this
#define STEER_TURN_DEG        20
#define STEER_SIGN            (+1)    // same direction as the tested pitch-steer code

enum State { ST_WAIT_FOR_PI, ST_READY, ST_STARTING, ST_RUNNING, ST_STOPPED };
static const char *STATE_NAME[] = { "WAIT_FOR_PI", "READY", "STARTING", "RUNNING", "STOPPED" };

static State    state = ST_WAIT_FOR_PI;
static uint32_t last_ready_ms = 0;
static uint32_t start_req_ms  = 0;
static bool     rec_failed    = false;
static TowerData tw;

static Servo steer;
static float sim_roll  = 0;          // simulated roll, deg (+ = right side down)
static int   steer_rel = 0;          // current steering, deg from centre

// =====================================================================
// Drive (stub until the rear motors arrive)
// =====================================================================
static void drive_start() { PI_SERIAL.println("LOG,drive start (stub)"); }
static void drive_stop()  { PI_SERIAL.println("LOG,drive stop (stub)"); }

// =====================================================================
// Steering from simulated tilt
// =====================================================================
static void steer_set(int rel)
{
  if (rel >  STEER_MAX_DEG) rel =  STEER_MAX_DEG;
  if (rel < -STEER_MAX_DEG) rel = -STEER_MAX_DEG;
  if (rel == steer_rel) return;
  steer_rel = rel;
  steer.write(STEER_CENTER_DEG + rel);
  PI_SERIAL.printf("LOG,steer %+d deg\n", rel);
}

static void update_steering(void)
{
  if (sim_roll > TILT_LIMIT_DEG)            steer_set( STEER_SIGN * STEER_TURN_DEG);
  else if (sim_roll < -TILT_LIMIT_DEG)      steer_set(-STEER_SIGN * STEER_TURN_DEG);
  else if (fabsf(sim_roll) < TILT_RELEASE_DEG) steer_set(0);
  // between 15 and 20 deg: keep what it's doing
}

static void handle_key(char k)
{
  if      (k == '1') sim_roll -= SIM_STEP_DEG;    // tilt left
  else if (k == '2') sim_roll += SIM_STEP_DEG;    // tilt right
  else if (k == '0') sim_roll = 0;
  else return;
  PI_SERIAL.printf("LOG,sim tilt %+.0f deg (%s)\n", sim_roll,
                   sim_roll > 0 ? "right side down" : sim_roll < 0 ? "left side down" : "level");
}

// =====================================================================
// State helpers
// =====================================================================
static void set_state(State s, const char *reason = nullptr)
{
  state = s;
  if (reason) PI_SERIAL.printf("STATE,%s,%s\n", STATE_NAME[s], reason);
  else        PI_SERIAL.printf("STATE,%s\n", STATE_NAME[s]);
}

static bool pi_alive(void)
{
  return last_ready_ms != 0 && (millis() - last_ready_ms) < PI_TIMEOUT_MS;
}

static void start_run(void)
{
  g_recording = false;
  rec_failed  = false;
  PI_SERIAL.println("START");
  start_req_ms = millis();
  set_state(ST_STARTING);
}

static void stop_run(const char *reason)
{
  drive_stop();
  tower_set_mode_idle();                       // LiDAR back to looking forward
  g_stop_moving = false;
  g_go_back     = false;
  PI_SERIAL.println("STOP");
  set_state(ST_STOPPED, reason);
}

// =====================================================================
// Pi messages
// =====================================================================
static void handle_line(const char *line)
{
  if (strcmp(line, "READY") == 0) {
    last_ready_ms = millis();
    if (state == ST_WAIT_FOR_PI) set_state(ST_READY);
  } else if (strcmp(line, "REC_ON") == 0) {
    g_recording = true;
  } else if (strcmp(line, "REC_OFF") == 0) {
    g_recording = false;
  } else if (strncmp(line, "REC_FAIL", 8) == 0) {
    rec_failed = true;
  } else if (strncmp(line, "KEY,", 4) == 0 && line[4] != '\0') {
    handle_key(line[4]);
  }
}

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

// =====================================================================
// Button: 1 = short press, 2 = long press, 0 = nothing
// =====================================================================
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

// =====================================================================
// Setup / loop
// =====================================================================
void setup()
{
  PI_SERIAL.begin(PI_BAUD);
  pinMode(REC_BUTTON_PIN, INPUT);              // external pull-up
  delay(300);

  steer.setPeriodHertz(50);
  steer.attach(SERVO_STEER_PIN, 500, 2400);
  steer.write(STEER_CENTER_DEG);

  PI_SERIAL.println("LOG,boot - keep still, calibrating");
  if (!tower_init()) PI_SERIAL.println("FAULT,tower_init (check IMU / LiDAR / stepper)");
  tower_set_mode_idle();                       // looking straight forward

  set_state(ST_WAIT_FOR_PI);
}

void loop()
{
  read_pi();
  update_steering();
  int btn = read_button();

  if (btn == 2) {                              // hold 3 s: stop + Pi shutdown
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
      if (btn == 1)   { stop_run("button"); break; }
      if (rec_failed) { PI_SERIAL.println("FAULT,no_usb_drive"); stop_run("no_usb_drive"); break; }
      if (g_recording) {                       // Pi is recording -> go
        g_stop_moving = false;
        g_go_back     = false;
        tower_set_mode_floor();                // tilt 20 deg down, watch the floor
        drive_start();
        set_state(ST_RUNNING);
      } else if (millis() - start_req_ms > START_ACK_TIMEOUT_MS) {
        PI_SERIAL.println("FAULT,no_rec_ack");
        stop_run("no_rec_ack");
      }
      break;

    case ST_RUNNING:
      if (btn == 1)    { stop_run("button");  break; }
      if (!pi_alive()) { stop_run("pi_lost"); break; }

      tower_read(tw);
      if (!tw.lidar.healthy) { PI_SERIAL.println("FAULT,lidar"); stop_run("lidar"); break; }
      if (!tw.imu.healthy)   { PI_SERIAL.println("FAULT,imu");   stop_run("imu");   break; }

      if (tw.floor_status == FLOOR_OBSTACLE) { // rise found -> end the run
        PI_SERIAL.printf("LOG,rise found: %.3f m high, %.3f m ahead\n",
                         tw.floor_height_m, tw.floor_ahead_m);
        stop_run("rise");
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
