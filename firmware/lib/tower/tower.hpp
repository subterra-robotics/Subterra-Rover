#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "imu.hpp"
#include "LiDAR.hpp"

// tower - the LiDAR scanning tower: MG90S tilt servo + NEMA 14 pan stepper
// (TMC2209) + TFmini-S. Owns the IMU and LiDAR background tasks and runs its
// own 100 Hz FreeRTOS task that:
//   - applies the requested mode (idle / floor / scan wall),
//   - in floor mode, turns LiDAR range + IMU pitch into a floor height and
//     classifies the floor ahead as clear / obstacle / dip / no return,
//   - on request, measures a blockage's distance, height and width.
// Callers only set modes and read snapshots; all motion happens in the task.

#define TOWER_READ_SUCCESS   0
#define TOWER_READ_ERROR    -1     // not initialised, or LiDAR not healthy

// Geometry (metres)
#define TOWER_HEIGHT_M      0.238f  // pipe floor -> tilt axle (tuned so flat floor reads flat at 20 deg)
#define LIDAR_FROM_AXIS     0.045f  // tilt axle -> LiDAR lens, along the beam

// Floor mode beam angle below horizontal: floor hit ~0.65 m ahead
#define FLOOR_TILT_DEG      20.0f

// What the tower is doing.
enum TowerMode {
  TOWER_MODE_IDLE = 0,       // LiDAR level, stepper released
  TOWER_MODE_FLOOR,          // LiDAR tilted down, watching the floor ahead
  TOWER_MODE_SCAN_WALL       // not implemented yet
};

// Debounced classification of the floor ahead in floor mode.
enum FloorStatus {
  FLOOR_UNKNOWN = 0,   // not in floor mode, servo settling, or LiDAR unhealthy
  FLOOR_CLEAR,
  FLOOR_OBSTACLE,      // something sticking up above the floor ahead
  FLOOR_DIP,           // floor drops away ahead
  FLOOR_NO_RETURN      // no usable return (hole, absorbing surface) -> treat as unsafe
};

// Progress of tower_measure_blockage().
enum BlockageState {
  BLOCK_IDLE = 0,      // no measurement run yet
  BLOCK_MEASURING,
  BLOCK_DONE,          // block_* fields are valid
  BLOCK_FAILED         // not in floor mode, no obstacle, or LiDAR lost the block
};

// One coherent snapshot of the tower, published by the tower task each
// cycle: sensor copies, actuator commands, floor result, blockage result
// and status flags, all from the same moment.
struct TowerData {
  // Sensor copies
  ImuData imu;
  LiDAR   lidar;

  // Actuators (commanded)
  float tilt_deg;          // beam angle below horizontal
  float pan_deg;           // stepper angle from centre, 0 = straight ahead

  // Floor mode result
  float distance_m;        // filtered range from the TILT AXLE (lens offset added)
  float expected_m;        // range from the axle to a clear floor (pitch-corrected)
  float floor_ahead_m;     // horizontal distance from the axle to the beam hit
  float floor_height_m;    // hit point vs floor: + = obstacle height, - = dip depth
  FloorStatus floor_status;

  // Blockage measurement
  BlockageState blockage_state;
  float block_distance_m;    // horizontal distance from the axle to the block face
  float block_height_m;      // height above the pipe floor
  float block_width_m;       // NAN if the pan stepper isn't available
  float block_offset_m;      // block centre sideways from straight ahead (+ = pan + side)
  bool  block_height_capped; // top not found before the tilt limit -> height is a MINIMUM

  // Status
  TowerMode mode;
  bool     settled;        // servo has had time to reach the commanded angle
  bool     stepper_ok;     // TMC2209 answered over UART at init
  bool     healthy;        // tower task running and LiDAR healthy
  uint32_t timestamp_ms;   // millis() of this update
};

// Brings up the IMU (with gyro calibration), LiDAR, tilt servo and pan
// stepper, starts their tasks and the tower task, and parks the tower in
// idle. The rover must be STILL for ~1 s. Returns false if any part failed
// (the tower still runs with what is available).
bool tower_init();

// Copies the latest TowerData. Returns TOWER_READ_SUCCESS, or
// TOWER_READ_ERROR if not initialised or the LiDAR is unhealthy.
int  tower_read(TowerData &data);

// Requests idle mode: LiDAR level (looking straight ahead), stepper released.
void tower_set_mode_idle();

// Requests floor mode: pan centred, tilt FLOOR_TILT_DEG down, floor ahead
// classified continuously. An obstacle sets g_stop_moving.
void tower_set_mode_floor();

// Requests wall-scan mode. Placeholder: enables the stepper, does no sweep yet.
void tower_set_mode_scan_wall();

// Flags shared with the drive / mission code (defined in src/flags.cpp).
extern volatile bool g_stop_moving;  // set by the tower when an obstacle is detected
extern volatile bool g_go_back;      // set when tower_measure_blockage() has finished

// Measures a blockage in front of the rover. Only works in floor mode with
// FLOOR_OBSTACLE showing; otherwise the result is BLOCK_FAILED. Blocks the
// caller until done (~5-15 s, 30 s timeout). Steps:
//   1) range to the block face at the floor angle
//   2) tilt up until the range jumps (beam passed over the top edge)
//   3) back to the last hit, re-measure, compute height (flat faces only)
//   4) aim at mid-height
//   5) pan both ways until the beam slips past each side -> width, offset
//   6) send "BLOCK,..." to the Pi (logged to blockages.csv)
//   7) return to floor posture and set g_go_back
// Results are in the next TowerData from tower_read().
void tower_measure_blockage();
