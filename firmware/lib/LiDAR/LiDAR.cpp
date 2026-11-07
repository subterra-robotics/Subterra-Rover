#include "LiDAR.hpp"
#include "config.hpp"
#include <Arduino.h>

// ---- TFmini-S data frame (9 bytes, standard cm format) ----
//  [0] 0x59  [1] 0x59  [2] Dist_L  [3] Dist_H  [4] Str_L  [5] Str_H
//  [6] Temp_L  [7] Temp_H  [8] checksum = low byte of sum of [0..7]
#define FRAME_HEADER      0x59
#define FRAME_LEN         9

// ---- Validity limits ----
#define MIN_STRENGTH      100      // below this the distance is unreliable
#define MAX_STRENGTH      65535    // saturated (too close / too reflective)
#define MAX_DISTANCE_CM   1200     // TFmini-S rated range

// ---- Health ----
#define ALIVE_TIMEOUT_MS  100      // no frame for this long = unhealthy (10 frames at 100 Hz)
#define WEAK_LIMIT        20       // weak frames in a row before unhealthy

#define LIDAR_UART        Serial2

// ---- Private state ----
static bool     initialized = false;
static uint8_t  buf[FRAME_LEN];
static uint8_t  idx = 0;
static uint32_t frames = 0;
static uint32_t cksum_errs = 0;
static uint16_t weak_frames = 0;
static uint32_t last_frame_ms = 0;

static portMUX_TYPE lidar_mux = portMUX_INITIALIZER_UNLOCKED;

// Sends a TFmini command frame (0x5A, length, id, payload...) followed by
// its checksum: the low byte of the sum of every byte before it.
static void send_command(const uint8_t *cmd, uint8_t len_without_checksum)
{
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len_without_checksum; i++) sum += cmd[i];
  LIDAR_UART.write(cmd, len_without_checksum);
  LIDAR_UART.write(sum);
}

// Byte-at-a-time frame parser. Hunts for the 0x59 0x59 header (which also
// skips command replies starting with 0x5A), collects 9 bytes and checks the
// checksum. Returns true when a valid frame is sitting in buf[].
static bool parse_byte(uint8_t b)
{
  if (idx < 2) {
    if (b == FRAME_HEADER) buf[idx++] = b;
    else idx = 0;
    return false;
  }

  buf[idx++] = b;
  if (idx < FRAME_LEN) return false;

  idx = 0;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < FRAME_LEN - 1; i++) sum += buf[i];
  if (sum != buf[FRAME_LEN - 1]) {
    cksum_errs++;
    return false;
  }
  return true;
}

// Opens UART2 with a 256-byte RX buffer, resets the counters and waits up to
// 500 ms for a first valid frame to prove the sensor is connected.
bool LiDAR_init()
{
  LIDAR_UART.setRxBufferSize(256);         // must be before begin()
  LIDAR_UART.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  idx = 0;
  frames = 0;
  cksum_errs = 0;
  weak_frames = 0;
  last_frame_ms = 0;
  initialized = true;

  uint32_t start = millis();
  while (millis() - start < 500) {
    while (LIDAR_UART.available()) {
      if (parse_byte((uint8_t)LIDAR_UART.read())) {
        last_frame_ms = millis();
        return true;
      }
    }
    delay(5);
  }
  return false;
}

// Parses every waiting byte. For each complete frame: decodes distance,
// strength and temperature, marks it valid only if strength and range are
// in limits, and keeps the last good distance on weak frames. Then updates
// the counters and the healthy flag (recent frame + not stuck weak).
int LiDAR_read(LiDAR &data)
{
  if (!initialized) {
    data.healthy = false;
    return LIDAR_ERR_NOT_INIT;
  }

  bool got_frame = false;

  while (LIDAR_UART.available()) {
    if (!parse_byte((uint8_t)LIDAR_UART.read())) continue;

    got_frame = true;
    frames++;
    last_frame_ms = millis();

    uint16_t dist = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t str  = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    uint16_t traw = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

    bool ok = (str >= MIN_STRENGTH) && (str != MAX_STRENGTH) &&
              (dist > 0) && (dist <= MAX_DISTANCE_CM);

    data.signal_strength = str;
    data.temp_c          = traw / 8.0f - 256.0f;
    data.valid           = ok;
    data.timestamp_ms    = last_frame_ms;

    if (ok) {
      data.distance_cm = dist;
      data.distance_m  = dist / 100.0f;
      weak_frames = 0;
    } else if (weak_frames < 0xFFFF) {
      weak_frames++;
    }
  }

  data.frame_count     = frames;
  data.checksum_errors = cksum_errs;
  data.weak_count      = weak_frames;
  data.healthy = (last_frame_ms != 0)
              && (millis() - last_frame_ms < ALIVE_TIMEOUT_MS)
              && (weak_frames < WEAK_LIMIT);

  if (!got_frame)  return LIDAR_NO_DATA;
  if (!data.valid) return LIDAR_WEAK_SIGNAL;
  return LIDAR_OK;
}

static LiDAR *task_out = nullptr;

// FreeRTOS task body: polls LiDAR_read() every 5 ms (sensor sends at 100 Hz)
// and publishes the result under the spinlock.
static void lidar_task(void *arg)
{
  LiDAR local = {};
  for (;;) {
    LiDAR_read(local);

    portENTER_CRITICAL(&lidar_mux);
    *task_out = local;
    portEXIT_CRITICAL(&lidar_mux);

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Launches lidar_task on core 1, priority 3.
bool LiDAR_start_task(LiDAR *out)
{
  if (!initialized || out == nullptr) return false;
  task_out = out;
  return xTaskCreatePinnedToCore(lidar_task, "lidar", 4096, NULL, 3, NULL, 1) == pdPASS;
}

// Copies the shared struct under the LiDAR spinlock.
void LiDAR_get_snapshot(const LiDAR *src, LiDAR *dst)
{
  portENTER_CRITICAL(&lidar_mux);
  *dst = *src;
  portEXIT_CRITICAL(&lidar_mux);
}

// Command 0x03: frame rate, little-endian Hz.
void LiDAR_set_frame_rate(uint16_t hz)
{
  uint8_t cmd[] = { 0x5A, 0x06, 0x03, (uint8_t)(hz & 0xFF), (uint8_t)(hz >> 8) };
  send_command(cmd, sizeof(cmd));
}

// Command 0x04: trigger one measurement.
void LiDAR_trigger()
{
  uint8_t cmd[] = { 0x5A, 0x04, 0x04 };
  send_command(cmd, sizeof(cmd));
}

// Command 0x07: enable (1) / disable (0) the output stream.
void LiDAR_enable_output(bool on)
{
  uint8_t cmd[] = { 0x5A, 0x05, 0x07, (uint8_t)(on ? 1 : 0) };
  send_command(cmd, sizeof(cmd));
}

// Command 0x11: save settings to the sensor's flash.
void LiDAR_save_settings()
{
  uint8_t cmd[] = { 0x5A, 0x04, 0x11 };
  send_command(cmd, sizeof(cmd));
}
