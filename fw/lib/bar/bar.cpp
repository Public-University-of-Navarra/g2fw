// MOCK of the barometer: it does not talk to the hardware.
// It simulates a slow climb to about 2 m and back (period 20 s), so the
// altitude changes, and it fails on purpose every 100th sample (FW_ERR_TIMEOUT)
// so the error path can be tested.
#include "bar.h"

#include <Arduino.h>
#include <math.h>

static const uint32_t kPeriodMs = 40;     // 25 Hz, the rate of the real sensor (contract)
static const uint32_t kErrorEvery = 100;  // one simulated error every 100 samples (4 s)
static const float kStartHpa = 1013.25f;  // pressure "measured" in bar_begin()
static const float kHpaPerMetre = 0.12f;  // near the ground, 1 m up = 0.12 hPa less
static const float kMinHpa = 260.0f;      // LPS28DFW range, used to validate the data
static const float kMaxHpa = 1260.0f;

static bool g_started = false;
static bool g_ready = false;
static bar_data_t g_last;
static float g_p0 = 0.0f;  // reference pressure, altitude 0
static uint32_t g_last_ms = 0;
static uint32_t g_count = 0;

// Altitude above the reference pressure p0 (international barometric formula),
// m.
static float altitudeFromPressure(float hpa, float p0) {
  return 44330.0f * (1.0f - powf(hpa / p0, 0.1903f));
}

int bar_begin() {
  g_p0 = kStartHpa;
  g_last_ms = millis();
  g_started = true;
  return FW_OK;
}

int bar_update() {
  if (!g_started) return FW_ERR_NOT_READY;

  uint32_t now = millis();
  if (now - g_last_ms < kPeriodMs) return FW_OK;  // not time yet: come back later, never wait
  g_last_ms = now;

  g_count++;
  if (g_count % kErrorEvery == 0) return FW_ERR_TIMEOUT;  // simulated failure

  float t = now / 1000.0f;
  float height_m = 1.0f - cosf(2.0f * PI * t / 20.0f);  // 0 to 2 m and back
  float hpa = g_p0 - kHpaPerMetre * height_m;
  if (hpa < kMinHpa || hpa > kMaxHpa) return FW_ERR_BAD_DATA;  // validate before using it

  g_last.hpa = hpa;
  g_last.altitude_m = altitudeFromPressure(hpa, g_p0);
  g_ready = true;
  return FW_OK;
}

int bar_get(bar_data_t *out) {
  if (out == nullptr) return FW_ERR_BAD_DATA;
  if (!g_ready) return FW_ERR_NOT_READY;  // no valid sample yet
  *out = g_last;
  return FW_OK;
}
