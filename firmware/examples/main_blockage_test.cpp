// Tower blockage test
// Floor mode watches the floor. When a rise is detected, the tower measures
// the block with tower_measure_blockage(). Remove the block to re-arm.
// Key 'm' = measure now. Rover STILL ~1 s at boot. Width needs 24 V on the TMC2209.

#include <Arduino.h>
#include "config.hpp"
#include "tower.hpp"

#define PRINT_PERIOD_MS  250

static const char *FLOOR_NAME[] = { "UNKNOWN", "CLEAR", "OBSTACLE", "DIP", "NO RETURN" };
static const char *BLOCK_NAME[] = { "IDLE", "MEASURING", "DONE", "FAILED" };

TowerData tw;
static bool armed = true;

static void print_block(void)
{
  Serial.printf("\n=== BLOCKAGE: %s ===\n", BLOCK_NAME[tw.blockage_state]);
  if (tw.blockage_state != BLOCK_DONE) return;
  Serial.printf("  distance ahead : %.3f m\n", tw.block_distance_m);
  Serial.printf("  height         : %.3f m %s\n", tw.block_height_m,
                tw.block_height_capped ? "(MINIMUM)" : "");
  if (isnan(tw.block_width_m)) Serial.println("  width          : n/a");
  else Serial.printf("  width          : %.3f m  (offset %+.3f m)\n",
                     tw.block_width_m, tw.block_offset_m);
  Serial.printf("  g_stop_moving=%d  g_go_back=%d\n\n", g_stop_moving, g_go_back);
}

static void measure(void)
{
  Serial.println(">> Measuring blockage...");
  tower_measure_blockage();
  tower_read(tw);
  print_block();
  armed = false;
}

void setup()
{
  Serial.begin(PI_BAUD);
  delay(500);
  Serial.println("\n=== Tower blockage test ===  (keep still)");
  if (!tower_init()) Serial.println("Tower: something failed to init");
  tower_set_mode_floor();
  delay(600);
  tower_read(tw);
  Serial.printf("IMU ok=%d  LiDAR ok=%d  stepper ok=%d\n\n",
                tw.imu.healthy, tw.lidar.healthy, tw.stepper_ok);
}

void loop()
{
  tower_read(tw);

  while (Serial.available()) if (Serial.read() == 'm') measure();

  if (armed && tw.floor_status == FLOOR_OBSTACLE) {
    Serial.println(">> Rise detected (g_stop_moving set)");
    measure();
  }
  if (!armed && tw.floor_status == FLOOR_CLEAR) {
    armed = true;
    g_stop_moving = false;
    g_go_back = false;
    Serial.println(">> Floor clear - re-armed\n");
  }

  static uint32_t last = 0;
  if (millis() - last >= PRINT_PERIOD_MS) {
    last = millis();
    Serial.printf("%-9s  h=%+.3f m  ahead=%.3f m  lidar=%.3f m  stop=%d\n",
                  FLOOR_NAME[tw.floor_status], tw.floor_height_m,
                  tw.floor_ahead_m, tw.lidar.distance_m, g_stop_moving);
  }
  delay(10);
}