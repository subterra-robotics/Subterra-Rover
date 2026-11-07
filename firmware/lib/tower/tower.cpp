#include "tower.hpp"
#include "config.hpp"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <TMCStepper.h>
#include <ESP32Servo.h>

// ---- Tuning ----
#define SERVO_SETTLE_MS     400      // ignore readings this long after the tilt moves
#define OBSTACLE_HEIGHT_M   0.04f    // > 4 cm above the floor = obstacle
#define DIP_DEPTH_M         0.04f    // > 4 cm below the floor = dip
#define DEBOUNCE_FRAMES     5        // same result this many frames in a row (50 ms) to switch
#define MEDIAN_N            5        // median filter length on the range
#define PITCH_LP_TAU_S      5.0f     // slow pitch average = pipe grade (not corrected for)
#define MIN_BEAM_DEG        3.0f     // beam too flat to compute a floor hit below this
#define TOWER_TASK_HZ       100
#define IMU_RATE_HZ         100
#define STEP_PULSE_US       150

// ---- Blockage measurement ----
#define BLOCK_TILT_STEP_DEG   1.0f   // tilt step while looking for the top edge
#define BLOCK_PAN_STEP_DEG    1.0f   // pan step while looking for the side edges
#define BLOCK_PAN_MAX_DEG     45.0f  // give up looking for a side past this
#define BLOCK_JUMP_M          0.10f  // range this much longer than the face = edge passed
#define BLOCK_TILT_SETTLE_MS  80     // servo settle per 1 deg step
#define BLOCK_PAN_SETTLE_MS   30     // stepper settle per step
#define BLOCK_SAMPLES         5      // frames per measurement (median)
#define BLOCK_SAMPLE_TIMEOUT  300    // ms to collect them
#define BLOCK_TIMEOUT_MS      30000  // tower_measure_blockage() gives up after this

#define DEG2RAD             0.01745329f

// ---- Hardware ----
static TMC2209Stepper driver(&Serial1, STEPPER_R_SENSE, DRIVER_ADDR);
static Servo tilt_servo;
static const float STEPS_PER_DEG = (200.0f * STEPPER_USTEPS) / 360.0f;

// ---- Shared state ----
static ImuData   s_imu;                      // kept updated by the IMU task
static LiDAR     s_lidar;                    // kept updated by the LiDAR task
static TowerData s_tower = {};               // written only by the tower task
static portMUX_TYPE tower_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile TowerMode req_mode = TOWER_MODE_IDLE;
static volatile bool req_measure = false;

static bool    initialized = false;
static bool    stepper_ok  = false;

// ---- Tower-task-only state ----
static int32_t  stepper_pos = 0;             // microsteps from centre (0 = power-up position)
static float    tilt_cmd = 0, pan_cmd = 0;   // deg
static uint32_t settle_until = 0;

static float    med_buf[MEDIAN_N];
static uint8_t  med_count = 0, med_idx = 0;
static float    pitch_lp = 0;
static bool     pitch_lp_init = false;
static uint32_t last_frame = 0;
static FloorStatus candidate = FLOOR_UNKNOWN;
static uint8_t     cand_count = 0;
static FloorStatus stable = FLOOR_UNKNOWN;

// =====================================================================
// Actuator helpers
// =====================================================================

// Tilt calibration table (measured 2026-10-29).
// TILT_CMD  = servo angle relative to SERVO_TILT_CENTER_DEG (raw = 90 + cmd)
// TILT_REAL = measured LiDAR angle, 0 = level, negative = looking DOWN
// The servo + linkage is non-linear, so the measured points are interpolated
// instead of using a single scale factor.
static const float TILT_CMD[]  = { -30, -20, -10,   0,   9,  20, 30, 41, 50, 60, 70, 80, 90 };   // 9 and 41 fine-tuned
static const float TILT_REAL[] = { -50, -45, -40, -30, -20, -10, -5,  0, 10, 20, 30, 40, 60 };
static const uint8_t TILT_N = sizeof(TILT_CMD) / sizeof(TILT_CMD[0]);

// Converts a real LiDAR angle to a servo command by linear interpolation
// between the calibration points. Clamps to the ends of the table.
static float tilt_real_to_cmd(float real)
{
  if (real <= TILT_REAL[0])          return TILT_CMD[0];
  if (real >= TILT_REAL[TILT_N - 1]) return TILT_CMD[TILT_N - 1];
  for (uint8_t i = 1; i < TILT_N; i++) {
    if (real <= TILT_REAL[i]) {
      float f = (real - TILT_REAL[i - 1]) / (TILT_REAL[i] - TILT_REAL[i - 1]);
      return TILT_CMD[i - 1] + f * (TILT_CMD[i] - TILT_CMD[i - 1]);
    }
  }
  return TILT_CMD[TILT_N - 1];
}

// Points the LiDAR deg_down below level (+20 = looking 20 deg down), clamped
// to +/-SERVO_TILT_MAX_DEG. Records the command and starts the settle timer
// so floor readings are ignored while the servo moves.
static void servo_set_tilt(float deg_down)
{
  if (deg_down >  SERVO_TILT_MAX_DEG) deg_down =  SERVO_TILT_MAX_DEG;
  if (deg_down < -SERVO_TILT_MAX_DEG) deg_down = -SERVO_TILT_MAX_DEG;
  tilt_cmd = deg_down;

  float cmd = tilt_real_to_cmd(-deg_down);
  int raw = (int)lroundf(SERVO_TILT_CENTER_DEG + cmd);
  tilt_servo.write(raw);
  PI_SERIAL.printf("LOG,tilt %.1f deg down -> raw %d (attached=%d)\n",   // DEBUG
                   deg_down, raw, tilt_servo.attached());
  settle_until = millis() + SERVO_SETTLE_MS;
}

// Enables (true) or releases (false) the stepper coils. EN is active LOW.
static void stepper_enable(bool on)
{
  digitalWrite(STEPPER_EN, on ? LOW : HIGH);
}

// Moves the pan stepper to an absolute angle by bit-banging STEP pulses.
// Open loop: position is counted from the power-up position (no homing).
// Blocking - only call from the tower task. No-op if the driver is absent.
static void stepper_goto_deg(float deg)
{
  pan_cmd = deg;
  if (!stepper_ok) return;

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

// Configures the stepper pins and the TMC2209 over UART (Serial1): register
// microstep select, STEPPER_MA RMS current, STEPPER_USTEPS microsteps,
// StealthChop with autoscale. Leaves the coils disabled. Returns false if the
// driver does not answer.
static bool stepper_setup(void)
{
  pinMode(STEPPER_EN, OUTPUT);
  stepper_enable(false);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);
  pinMode(STEPPER_DIAG, INPUT);

  Serial1.begin(115200, SERIAL_8N1, STEPPER_UART_RX, STEPPER_UART_TX);
  driver.begin();
  if (driver.test_connection() != 0) return false;

  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.I_scale_analog(false);
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(STEPPER_MA);
  driver.microsteps(STEPPER_USTEPS);
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);
  driver.SGTHRS(0);
  return true;
}

// =====================================================================
// Floor-mode helpers
// =====================================================================

// Adds one range sample to the MEDIAN_N ring buffer.
static void median_push(float v)
{
  med_buf[med_idx] = v;
  med_idx = (med_idx + 1) % MEDIAN_N;
  if (med_count < MEDIAN_N) med_count++;
}

// Returns the median of the samples currently in the ring buffer.
static float median_get(void)
{
  float tmp[MEDIAN_N];
  for (uint8_t i = 0; i < med_count; i++) tmp[i] = med_buf[i];
  for (uint8_t i = 1; i < med_count; i++) {
    float x = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > x) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = x;
  }
  return tmp[med_count / 2];
}

// Clears the median filter and the debouncer, so the floor status restarts
// from FLOOR_UNKNOWN (after a mode change, tilt move or unhealthy LiDAR).
static void floor_reset(void)
{
  med_count = 0;
  med_idx = 0;
  candidate = FLOOR_UNKNOWN;
  cand_count = 0;
  stable = FLOOR_UNKNOWN;
}

// Debouncer: the published floor status only changes after the same raw
// classification has been seen DEBOUNCE_FRAMES times in a row.
static void debounce(FloorStatus raw)
{
  if (raw == candidate) {
    if (cand_count < 255) cand_count++;
  } else {
    candidate = raw;
    cand_count = 1;
  }
  if (cand_count >= DEBOUNCE_FRAMES) stable = candidate;
}

// Floor detection for one new LiDAR frame.
// The rover's pitch is split into a slow average (the pipe grade, which the
// floor follows anyway) and the fast remainder (bumps), and only the fast
// part corrects the beam angle. The median-filtered range from the tilt axle
// is then turned into a hit height relative to the floor:
//   height = TOWER_HEIGHT_M - range * sin(beam)
// and classified as clear / obstacle / dip, or no return if the frame was
// invalid. Results are written into t and fed to the debouncer.
static void floor_update(TowerData &t, const ImuData &imu, const LiDAR &lid)
{
  float dt = 1.0f / TOWER_TASK_HZ;
  float pitch_rel = 0;
  if (imu.healthy) {
    if (!pitch_lp_init) { pitch_lp = imu.pitch_deg; pitch_lp_init = true; }
    pitch_lp += (imu.pitch_deg - pitch_lp) * (dt / PITCH_LP_TAU_S);
    pitch_rel = imu.pitch_deg - pitch_lp;
  }

  float beam_deg = tilt_cmd - pitch_rel;     // nose up -> beam less steep
  if (beam_deg < MIN_BEAM_DEG) {
    debounce(FLOOR_UNKNOWN);
    return;
  }
  float s = sinf(beam_deg * DEG2RAD);
  float c = cosf(beam_deg * DEG2RAD);
  t.expected_m = TOWER_HEIGHT_M / s;

  if (!lid.valid) {
    debounce(FLOOR_NO_RETURN);
    return;
  }

  median_push(lid.distance_m);
  float r = median_get() + LIDAR_FROM_AXIS;

  t.distance_m     = r;
  t.floor_ahead_m  = r * c;
  t.floor_height_m = TOWER_HEIGHT_M - r * s;

  if      (t.floor_height_m >  OBSTACLE_HEIGHT_M) debounce(FLOOR_OBSTACLE);
  else if (t.floor_height_m < -DIP_DEPTH_M)       debounce(FLOOR_DIP);
  else                                            debounce(FLOOR_CLEAR);
}

// =====================================================================
// Blockage measurement helpers (run inside the tower task)
// =====================================================================

// Publishes t as the shared tower snapshot under the tower spinlock.
static void publish(const TowerData &t)
{
  portENTER_CRITICAL(&tower_mux);
  s_tower = t;
  portEXIT_CRITICAL(&tower_mux);
}

// Collects up to BLOCK_SAMPLES new valid LiDAR frames (BLOCK_SAMPLE_TIMEOUT
// ms max) and returns their median as range from the tilt axle. Returns -1
// if fewer than half arrived, which callers treat as "beam passed the edge".
static float lidar_measure_axle(void)
{
  float v[BLOCK_SAMPLES];
  uint8_t k = 0;
  LiDAR l;
  LiDAR_get_snapshot(&s_lidar, &l);
  uint32_t lastf = l.frame_count;
  uint32_t start = millis();

  while (k < BLOCK_SAMPLES && millis() - start < BLOCK_SAMPLE_TIMEOUT) {
    vTaskDelay(pdMS_TO_TICKS(5));
    LiDAR_get_snapshot(&s_lidar, &l);
    if (l.frame_count == lastf) continue;
    lastf = l.frame_count;
    if (l.valid) v[k++] = l.distance_m;
  }
  if (k < (BLOCK_SAMPLES + 1) / 2) return -1.0f;

  for (uint8_t i = 1; i < k; i++) {
    float x = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[k / 2] + LIDAR_FROM_AXIS;
}

// Pans in direction dir (+1 / -1) from centre, one step at a time, until the
// range jumps past what a flat face square to the rover would give (or no
// return). The edge is taken halfway between the last hit and the first
// miss. Returns its sideways offset in metres, or NAN if no edge was found
// within BLOCK_PAN_MAX_DEG.
static float find_side_edge(float dir, float x_face, float tilt_down)
{
  float c_tilt = cosf(tilt_down * DEG2RAD);
  float last_ok = 0;

  for (float p = BLOCK_PAN_STEP_DEG; p <= BLOCK_PAN_MAX_DEG; p += BLOCK_PAN_STEP_DEG) {
    stepper_goto_deg(dir * p);
    vTaskDelay(pdMS_TO_TICKS(BLOCK_PAN_SETTLE_MS));

    float r = lidar_measure_axle();
    float expected = x_face / (cosf(p * DEG2RAD) * c_tilt);
    if (r < 0 || r > expected + BLOCK_JUMP_M) {
      float edge = last_ok + BLOCK_PAN_STEP_DEG / 2;
      return dir * x_face * tanf(edge * DEG2RAD);
    }
    last_ok = p;
  }
  return NAN;
}

// Runs steps 1-7 of tower_measure_blockage() (see tower.hpp) and fills the
// block_* fields of t. Assumes a flat, vertical face square to the rover.
// Leaves the tower back in floor posture and sets g_go_back on success.
static void run_blockage(TowerData &t)
{
  t.blockage_state      = BLOCK_MEASURING;
  t.block_height_capped = false;
  t.block_width_m       = NAN;
  t.block_offset_m      = NAN;
  publish(t);

  float tilt0 = tilt_cmd;

  // 1) distance to the block face at the floor angle
  float r0 = lidar_measure_axle();
  if (r0 < 0) { t.blockage_state = BLOCK_FAILED; return; }
  float x_face = r0 * cosf(tilt0 * DEG2RAD);

  // 2) tilt up until the range jumps (beam passes over the top edge)
  float last_ok = tilt0;
  bool  found_top = false;
  for (float a = tilt0 - BLOCK_TILT_STEP_DEG; a >= -SERVO_TILT_MAX_DEG; a -= BLOCK_TILT_STEP_DEG) {
    servo_set_tilt(a);
    vTaskDelay(pdMS_TO_TICKS(BLOCK_TILT_SETTLE_MS));

    float r = lidar_measure_axle();
    float expected = x_face / cosf(a * DEG2RAD);
    if (r < 0 || r > expected + BLOCK_JUMP_M) { found_top = true; break; }
    last_ok = a;
  }

  // 3) back to the last angle that hit the face, re-measure, compute height
  servo_set_tilt(last_ok);
  vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
  float r_edge = lidar_measure_axle();
  if (r_edge > 0) x_face = 0.5f * (x_face + r_edge * cosf(last_ok * DEG2RAD));

  float edge_deg = found_top ? last_ok - BLOCK_TILT_STEP_DEG / 2 : last_ok;
  float height = TOWER_HEIGHT_M - x_face * tanf(edge_deg * DEG2RAD);
  t.block_distance_m    = x_face;
  t.block_height_m      = height;
  t.block_height_capped = !found_top;

  // 4) aim at the middle of the block's height
  float mid_down = atanf((TOWER_HEIGHT_M - height / 2) / x_face) / DEG2RAD;
  servo_set_tilt(mid_down);
  vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));

  // 5) pan both ways to find the sides -> width
  if (stepper_ok) {
    float x_mid = x_face;
    float y_pos = find_side_edge(+1, x_mid, tilt_cmd);
    stepper_goto_deg(0);
    float y_neg = find_side_edge(-1, x_mid, tilt_cmd);
    stepper_goto_deg(0);
    if (!isnan(y_pos) && !isnan(y_neg)) {
      t.block_width_m  = y_pos - y_neg;
      t.block_offset_m = 0.5f * (y_pos + y_neg);
    }
  }

  // 6) send to the Pi
  PI_SERIAL.printf("BLOCK,%lu,%.3f,%.3f,%.3f,%.3f,%d\n",
                   (unsigned long)millis(), t.block_distance_m, t.block_height_m,
                   t.block_width_m, t.block_offset_m, t.block_height_capped ? 1 : 0);

  servo_set_tilt(tilt0);
  floor_reset();

  // 7) let the rover go back
  t.blockage_state = BLOCK_DONE;
  g_go_back = true;
}

// =====================================================================
// Mode handling (runs inside the tower task)
// =====================================================================

// One-time actions on entering a mode: resets the floor filters and moves
// the servo / stepper into that mode's posture. In idle the stepper is
// released, so it must not be turned by hand (its position is assumed).
static void enter_mode(TowerMode m)
{
  floor_reset();
  pitch_lp_init = false;

  switch (m) {
    case TOWER_MODE_IDLE:
      servo_set_tilt(0);
      stepper_enable(false);
      break;

    case TOWER_MODE_FLOOR:
      if (stepper_ok) stepper_enable(true);
      stepper_goto_deg(0);
      servo_set_tilt(FLOOR_TILT_DEG);
      break;

    case TOWER_MODE_SCAN_WALL:
      // TODO: scan-wall setup (starting tilt / pan, sweep pattern)
      if (stepper_ok) stepper_enable(true);
      break;
  }
}

// The tower task (100 Hz, core 1). Each cycle it applies any mode change,
// snapshots the IMU and LiDAR, runs floor detection on new frames, raises
// g_stop_moving on an obstacle, services a pending blockage measurement,
// and publishes a fresh TowerData.
static void tower_task(void *arg)
{
  int        cur = -1;                   // forces enter_mode() on the first loop
  TowerData  t = {};
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    TowerMode want = req_mode;
    if ((int)want != cur) {
      enter_mode(want);
      cur = (int)want;
    }

    imu_get_snapshot(&s_imu, &t.imu);
    LiDAR_get_snapshot(&s_lidar, &t.lidar);

    bool new_frame = (t.lidar.frame_count != last_frame);
    last_frame = t.lidar.frame_count;
    t.settled = ((int32_t)(millis() - settle_until) >= 0);

    switch (cur) {
      case TOWER_MODE_FLOOR:
        if (!t.lidar.healthy || !t.settled) {
          floor_reset();
        } else if (new_frame) {
          floor_update(t, t.imu, t.lidar);
        }
        t.floor_status = stable;
        if (stable == FLOOR_OBSTACLE) g_stop_moving = true;

        if (req_measure) {
          if (stable == FLOOR_OBSTACLE && t.settled) run_blockage(t);
          else t.blockage_state = BLOCK_FAILED;
          req_measure = false;
        }
        break;

      case TOWER_MODE_SCAN_WALL:
        // TODO: scan-wall sweep + processing
        t.floor_status = FLOOR_UNKNOWN;
        break;

      default:
        t.floor_status = FLOOR_UNKNOWN;
        break;
    }

    if (req_measure && cur != TOWER_MODE_FLOOR) {
      t.blockage_state = BLOCK_FAILED;
      req_measure = false;
    }

    t.tilt_deg     = tilt_cmd;
    t.pan_deg      = pan_cmd;
    t.mode         = (TowerMode)cur;
    t.stepper_ok   = stepper_ok;
    t.healthy      = t.lidar.healthy;
    t.timestamp_ms = millis();

    publish(t);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / TOWER_TASK_HZ));
  }
}

// =====================================================================
// Public
// =====================================================================

// Initialises I2C + IMU (calibrate, start task), LiDAR (task starts even
// without frames so a late sensor still works), the tilt servo at level and
// the TMC2209, then launches the tower task (core 1, priority 2).
bool tower_init()
{
  bool ok = true;

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);
  if (imu_init()) {
    imu_calibrate();
    imu_start_task(&s_imu, IMU_RATE_HZ);
  } else {
    ok = false;
  }

  if (!LiDAR_init()) ok = false;
  LiDAR_start_task(&s_lidar);

  tilt_servo.setPeriodHertz(50);
  tilt_servo.attach(SERVO_TILT_PIN, 500, 2400);
  servo_set_tilt(0);

  stepper_ok = stepper_setup();
  if (!stepper_ok) ok = false;

  initialized = true;
  xTaskCreatePinnedToCore(tower_task, "tower", 4096, NULL, 2, NULL, 1);
  return ok;
}

// Copies the shared snapshot under the tower spinlock.
int tower_read(TowerData &data)
{
  if (!initialized) return TOWER_READ_ERROR;

  portENTER_CRITICAL(&tower_mux);
  data = s_tower;
  portEXIT_CRITICAL(&tower_mux);

  return data.healthy ? TOWER_READ_SUCCESS : TOWER_READ_ERROR;
}

// Mode requests: only set req_mode; the tower task applies the change.
void tower_set_mode_idle()      { req_mode = TOWER_MODE_IDLE; }
void tower_set_mode_floor()     { req_mode = TOWER_MODE_FLOOR; }
void tower_set_mode_scan_wall() { req_mode = TOWER_MODE_SCAN_WALL; }

// Raises req_measure, then polls the shared snapshot every 20 ms until the
// tower task has cleared the request and finished measuring, or
// BLOCK_TIMEOUT_MS passes.
void tower_measure_blockage()
{
  if (!initialized) return;

  req_measure = true;
  uint32_t start = millis();

  while (millis() - start < BLOCK_TIMEOUT_MS) {
    delay(20);
    TowerData d;
    portENTER_CRITICAL(&tower_mux);
    d = s_tower;
    portEXIT_CRITICAL(&tower_mux);
    if (!req_measure && d.blockage_state != BLOCK_MEASURING) return;
  }
}
