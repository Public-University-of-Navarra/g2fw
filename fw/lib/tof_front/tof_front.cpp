// tof_front MOCK: fake VL53L8CX that never touches the hardware (no I2C, no pins).
//
// It simulates the real sensor in 4x4 mode at 60 Hz:
//   - an obstacle in the 4 central zones (5, 6, 9, 10) that comes closer from 2000 mm to 300 mm
//     and moves away again, in a 20 s cycle;
//   - the zones around it see a wall 500 mm behind the obstacle;
//   - the top row (zones 0-3) sees nothing: 4000 mm = no target (path clear).
// Every MOCK_ERROR_EVERY frames (~1 per second) update() returns FW_ERR_BAD_DATA, as a corrupted
// frame of the real sensor would, and the last valid sample is kept.
//
// The real driver will replace this file; tof_front.h does not change.
#include "tof_front.h"

#include <Arduino.h>  // millis()

namespace {

constexpr uint8_t ZONES = 16;
constexpr uint16_t NO_TARGET_MM = 4000;        // contract: no target = 4000
constexpr uint32_t FRAME_PERIOD_MS = 17;       // ~60 Hz, like the real 4x4 mode
constexpr uint32_t MOCK_ERROR_EVERY = 60;      // one bad frame every 60 (~1 per second)

constexpr uint16_t OBSTACLE_FAR_MM = 2000;
constexpr uint16_t OBSTACLE_NEAR_MM = 300;
constexpr uint32_t CYCLE_MS = 20000;           // far -> near -> far
constexpr uint16_t WALL_BEHIND_MM = 500;       // the wall is 500 mm behind the obstacle

bool started = false;
bool has_sample = false;
uint32_t last_frame_ms = 0;
uint32_t frame_count = 0;
tof_front_data_t last_valid = {};

bool is_center(uint8_t zone) { return zone == 5 || zone == 6 || zone == 9 || zone == 10; }

// Triangle wave: OBSTACLE_FAR_MM at t = 0, OBSTACLE_NEAR_MM at CYCLE_MS / 2, back at CYCLE_MS.
uint16_t obstacle_mm(uint32_t now_ms) {
  uint32_t t = now_ms % CYCLE_MS;
  uint32_t half = CYCLE_MS / 2;
  uint32_t span = OBSTACLE_FAR_MM - OBSTACLE_NEAR_MM;
  uint32_t travelled = (t < half) ? (span * t) / half : (span * (CYCLE_MS - t)) / half;
  return (uint16_t)(OBSTACLE_FAR_MM - travelled);
}

}  // namespace

int tof_front_begin() {
  // The real driver uploads the sensor firmware here (~2 s). The mock is ready at once.
  started = true;
  has_sample = false;
  frame_count = 0;
  last_frame_ms = millis();
  return FW_OK;
}

int tof_front_update() {
  if (!started) return FW_ERR_NOT_READY;

  uint32_t now = millis();
  if (now - last_frame_ms < FRAME_PERIOD_MS) return FW_OK;  // no new frame yet: nothing to do
  last_frame_ms = now;
  frame_count++;

  // Error path: a corrupted frame is discarded and the last valid sample is kept.
  if (frame_count % MOCK_ERROR_EVERY == 0) return FW_ERR_BAD_DATA;

  tof_front_data_t frame;
  uint16_t obstacle = obstacle_mm(now);
  uint16_t wall = obstacle + WALL_BEHIND_MM;
  frame.min_mm = NO_TARGET_MM;

  for (uint8_t zone = 0; zone < ZONES; zone++) {
    uint16_t d;
    if (zone < 4) {
      d = NO_TARGET_MM;                        // top row: nothing in range
    } else if (is_center(zone)) {
      d = obstacle + (zone % 3) * 5;           // a few mm of difference between zones, like the real one
    } else {
      d = wall;
    }
    if (d > NO_TARGET_MM) d = NO_TARGET_MM;    // validate range (0 to 4000)
    frame.zone_mm[zone] = d;
    if (d < frame.min_mm) frame.min_mm = d;
  }

  last_valid = frame;
  has_sample = true;
  return FW_OK;
}

int tof_front_get(tof_front_data_t *out) {
  if (out == nullptr) return FW_ERR_BAD_DATA;
  if (!has_sample) return FW_ERR_NOT_READY;  // *out untouched
  *out = last_valid;
  return FW_OK;
}
