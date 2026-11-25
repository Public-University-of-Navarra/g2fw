// MOCK of the camera link: it does not read DSP_SYNC.
// It simulates the pulse the DSP sends at the start of every image of a PAL camera (25 images/s).
// In the last 2 s of every 20 s the "DSP" goes silent, so after 1 s without pulses
// camera_update() returns FW_ERR_TIMEOUT and the error path can be tested.
#include "camera.h"

#include <Arduino.h>

static const uint32_t kFramePeriodUs = 40000;  // 25 images/s (PAL camera)
static const uint32_t kTimeoutUs = 1000000;    // contract: error if no pulse in the last 1 s
static const uint32_t kCycleMs = 20000;        // every 20 s...
static const uint32_t kSilentMs = 2000;        // ...the DSP sends no pulse for 2 s

static bool g_started = false;
static uint32_t g_begin_us = 0;
static camera_data_t g_last = {0, 0};

int camera_begin() {
  g_begin_us = micros();
  g_started = true;
  return FW_OK;
}

int camera_update() {
  if (!g_started) return FW_ERR_NOT_READY;

  uint32_t now_us = micros();
  uint32_t ref_us = (g_last.frames > 0) ? g_last.last_sync_us : g_begin_us;  // last pulse, or boot
  bool dsp_silent = (millis() % kCycleMs) >= (kCycleMs - kSilentMs);

  if (!dsp_silent && now_us - ref_us >= kFramePeriodUs) {  // a new image started: "pulse"
    g_last.frames++;
    g_last.last_sync_us = now_us;
    return FW_OK;
  }
  if (now_us - ref_us > kTimeoutUs) return FW_ERR_TIMEOUT;  // no pulse in the last second
  return FW_OK;
}

int camera_get(camera_data_t *out) {
  if (out == nullptr) return FW_ERR_BAD_DATA;
  if (g_last.frames == 0) return FW_ERR_NOT_READY;  // no image yet
  *out = g_last;
  return FW_OK;
}
