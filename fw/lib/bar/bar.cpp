// REAL driver of the barometer: LPS28DFW on I2C_SENS (Wire), address 0x5C, with the SparkFun
// library (declared in fw/platformio.ini). Contract: fw/README.md, section "bar".
//
// Adapted from test 03 of the validation firmware (19/11/2026): same configuration (1260 hPa full
// scale, average of 16, low-pass filter ODR/4) but at 25 Hz, the rate of the contract.
// The sensor is polled through its status register (like test 03), so the INT_ALT line is not
// needed to read it.
//
// Known limitation, to check on the real board: the altitude zero is the first valid sample read
// in bar_begin() (contract). In Instrumentacion Avanzada the internal filter needed ~3 s to
// settle, so the first seconds may show an offset of some centimetres.
#include "bar.h"

#include <Arduino.h>
#include <SparkFun_LPS28DFW_Arduino_Library.h>
#include <Wire.h>
#include <board_pins.h>
#include <math.h>

namespace {

constexpr float kMinHpa = 260.0f;  // LPS28DFW range in this full scale (validate input)
constexpr float kMaxHpa = 1260.0f;
constexpr uint32_t kPollMs = 10;          // read the status at most every 10 ms (data: 25 Hz)
constexpr uint32_t kDataTimeoutMs = 500;  // no new sample in 0.5 s -> FW_ERR_TIMEOUT
constexpr uint32_t kFirstSampleTimeoutMs = 500;  // bar_begin(): max wait for the altitude zero

LPS28DFW sensor;

int g_begin_err = FW_ERR_NOT_READY;  // result of bar_begin(); bar_update() repeats it if it failed
bool g_ready = false;
float g_p0 = NAN;  // reference pressure, altitude 0
bar_data_t g_last = {0.0f, 0.0f};
uint32_t g_last_poll_ms = 0;
uint32_t g_last_data_ms = 0;

// Altitude above the reference pressure p0 (international barometric formula), m.
float altitudeFromPressure(float hpa, float p0) {
  return 44330.0f * (1.0f - powf(hpa / p0, 0.1903f));
}

// Quick presence check: the library has no timeout of its own.
bool sensorAcks() {
  Wire.beginTransmission(ADDR_LPS28DFW);
  return Wire.endTransmission() == 0;
}

// Reads one sample if the sensor has a new one. *got = false if there is none yet.
int readSample(float *hpa, bool *got) {
  *got = false;
  lps28dfw_stat_t status;
  if (sensor.getStatus(&status) != LPS28DFW_OK) return FW_ERR_NOT_FOUND;  // no answer on I2C
  if (!status.drdy_pres) return FW_OK;                                    // no new sample yet
  if (sensor.getSensorData() != LPS28DFW_OK) return FW_ERR_BAD_DATA;
  float p = sensor.data.pressure.hpa;
  if (!(p >= kMinHpa && p <= kMaxHpa)) return FW_ERR_BAD_DATA;  // out of range (also NaN)
  *hpa = p;
  *got = true;
  return FW_OK;
}

int beginImpl() {
  if (!sensorAcks()) return FW_ERR_NOT_FOUND;  // e.g. Wokwi: there is no LPS28DFW
  // Returns LPS28DFW_OK (0) if WHO_AM_I is 0xB4: another device at 0x5C is "not found" for us.
  if (sensor.begin(ADDR_LPS28DFW, Wire) != LPS28DFW_OK) return FW_ERR_NOT_FOUND;

  lps28dfw_md_t mode = {};
  mode.fs = LPS28DFW_1260hPa;
  mode.odr = LPS28DFW_25Hz;
  mode.avg = LPS28DFW_16_AVG;
  mode.lpf = LPS28DFW_LPF_ODR_DIV_4;
  if (sensor.setModeConfig(&mode) != LPS28DFW_OK) return FW_ERR_NOT_FOUND;

  // Altitude zero: the first valid sample (contract: "the pressure read in bar_begin()").
  uint32_t t0 = millis();
  float p = NAN;
  bool got = false;
  while (!got && millis() - t0 < kFirstSampleTimeoutMs) {
    int err = readSample(&p, &got);
    if (err != FW_OK) return err;
    if (!got) delay(5);  // setup-only: next sample in <= 40 ms
  }
  if (!got) return FW_ERR_TIMEOUT;

  g_p0 = p;
  g_last.hpa = p;
  g_last.altitude_m = 0.0f;
  g_ready = true;
  g_last_poll_ms = g_last_data_ms = millis();
  return FW_OK;
}

}  // namespace

int bar_begin() {
  g_ready = false;
  g_begin_err = beginImpl();
  return g_begin_err;
}

int bar_update() {
  if (g_begin_err != FW_OK) return g_begin_err;  // keep reporting why begin() failed

  uint32_t now = millis();
  if (now - g_last_poll_ms < kPollMs) return FW_OK;  // not time yet: never wait
  g_last_poll_ms = now;

  float p = NAN;
  bool got = false;
  int err = readSample(&p, &got);
  if (err != FW_OK) return err;
  if (!got) return (now - g_last_data_ms > kDataTimeoutMs) ? FW_ERR_TIMEOUT : FW_OK;

  g_last.hpa = p;
  g_last.altitude_m = altitudeFromPressure(p, g_p0);
  g_last_data_ms = now;
  return FW_OK;
}

int bar_get(bar_data_t *out) {
  if (out == nullptr) return FW_ERR_BAD_DATA;
  if (!g_ready) return FW_ERR_NOT_READY;  // no valid sample yet
  *out = g_last;
  return FW_OK;
}
