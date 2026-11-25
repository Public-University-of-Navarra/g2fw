// tof_inf: bottom distance sensor, VL53L1X (real driver). Contract:
// fw/README.md. The header is the same as the mock one. Polling only (INT pin
// not used), never waits in update().
#include "tof_inf.h"

#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>

// ---- sensor settings ----
// Short distance mode (up to ~1.3 m, robust against ambient light), 33 ms
// timing budget, one measurement every 40 ms (25 Hz). Set TOF_INF_SHORT_MODE to
// 0 for the default long mode (100 ms budget, up to ~4 m in the dark).
#ifndef TOF_INF_SHORT_MODE
#define TOF_INF_SHORT_MODE 1
#endif
#if TOF_INF_SHORT_MODE
static const uint32_t INTER_MEAS_MS = 40;
#else
static const uint32_t INTER_MEAS_MS = 100;
#endif

static const uint16_t MAX_MM = 4000;          // contract range: 0 to 4000 mm
static const uint32_t POLL_MS = 5;            // how often update() asks "is data ready?"
static const uint32_t DATA_TIMEOUT_MS = 500;  // no new measurement for this long -> timeout
static const uint8_t ADDR = ADDR_VL53L1X;     // 7-bit address (0x29)

// ---- registers (16-bit index, MSB first) ----
static const uint16_t REG_SOFT_RESET_BOOT = 0x00E5;  // != 0 when the sensor has booted
static const uint16_t REG_MODEL_ID = 0x010F;         // 0xEA 0xCC
static const uint16_t REG_GPIO_HV_MUX_CTRL = 0x0030;
static const uint16_t REG_GPIO_TIO_HV_STATUS = 0x0031;
static const uint16_t REG_SYSTEM_INT_CLEAR = 0x0086;
static const uint16_t REG_SYSTEM_START = 0x0087;
static const uint16_t REG_RESULT = 0x0089;  // 17 bytes
static const uint16_t REG_OSC_CALIB = 0x00DE;
static const uint16_t REG_INTER_MEAS = 0x006C;
static const uint16_t REG_CONFIG_FIRST = 0x002D;  // default configuration: 0x2D..0x87
static const uint8_t RESULT_LEN = 17;

// ULD default configuration (91 bytes, registers 0x2D..0x87).
// 0x2E and 0x2F are set to 0x01 in begin(): I/O at 2.8 V on the VL53L1X-SATEL.
static const uint8_t CONFIG_LEN = 91;
static const uint8_t CONFIG[CONFIG_LEN] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xff, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xcc, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00, 0x80, 0x08, 0xb8, 0x00, 0x00, 0x00,
    0x00, 0x0f, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
    0x00, 0x02, 0xc7, 0xff, 0x9b, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};

// ---- state ----
static bool g_begun = false;               // begin() finished OK
static int g_beginErr = FW_ERR_NOT_READY;  // result of begin(); returned by update() if it failed
static bool g_ready = false;               // there is a valid sample
static uint16_t g_last = 0;                // last valid distance, mm
static uint8_t g_pol = 1;                  // interrupt polarity: data ready when bit0 == g_pol
static uint32_t g_tPoll = 0;               // last time we asked the sensor
static uint32_t g_tData = 0;               // last time a measurement arrived

// ---- I2C helpers (bounded buffers, every result checked) ----
static bool writeReg(uint16_t reg, const uint8_t *data, uint8_t n) {
  if (n > 16) return false;  // one chunk fits easily in the Wire buffer
  Wire.beginTransmission(ADDR);
  Wire.write((uint8_t)(reg >> 8));
  Wire.write((uint8_t)(reg & 0xFF));
  for (uint8_t i = 0; i < n; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

static bool writeByte(uint16_t reg, uint8_t v) { return writeReg(reg, &v, 1); }

static bool readReg(uint16_t reg, uint8_t *out, uint8_t n) {
  if (out == nullptr || n == 0 || n > RESULT_LEN) return false;
  if (!writeReg(reg, nullptr, 0)) return false;  // set the register pointer
  if (Wire.requestFrom((int)ADDR, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!Wire.available()) return false;
    out[i] = (uint8_t)Wire.read();
  }
  return true;
}

// Writes a table to consecutive registers, in chunks of 16 bytes.
static bool writeTable(uint16_t reg, const uint8_t *data, uint8_t n) {
  for (uint8_t i = 0; i < n; i += 16) {
    uint8_t len = (n - i < 16) ? (n - i) : 16;
    if (!writeReg(reg + i, data + i, len)) return false;
  }
  return true;
}

// Sensor status (raw & 0x1F) -> 0 when the distance is valid (same table as
// ST's ULD).
static uint8_t rangeStatus(uint8_t raw) {
  static const uint8_t MAP[24] = {255, 255, 255, 5,   2,   4,   1,  7, 3,   0,   255, 255,
                                  9,   13,  255, 255, 255, 255, 10, 6, 255, 255, 11,  12};
  raw &= 0x1F;
  return (raw < 24) ? MAP[raw] : 255;
}

// ---- contract functions ----
int tof_inf_begin() {
  g_begun = false;
  g_ready = false;
  g_beginErr = FW_ERR_NOT_FOUND;

  // Power cycle through XSHUT: LOW = off, HIGH = on (the configuration is lost
  // when it is off).
  pinMode(PIN_TOF_INF_XSHUT, OUTPUT);
  digitalWrite(PIN_TOF_INF_XSHUT, LOW);
  delay(2);  // setup-only
  digitalWrite(PIN_TOF_INF_XSHUT, HIGH);
  delay(2);  // setup-only

  // Wait for boot (up to ~1.2 ms in the datasheet; we allow 100 ms).
  uint8_t b = 0;
  bool answered = false;
  uint32_t t0 = millis();
  while (millis() - t0 < 100) {
    if (readReg(REG_SOFT_RESET_BOOT, &b, 1)) {
      answered = true;
      if (b != 0) break;
    }
    delay(1);  // setup-only
  }
  if (!answered) return g_beginErr = FW_ERR_NOT_FOUND;  // nobody answers at 0x29
  if (b == 0) return g_beginErr = FW_ERR_TIMEOUT;       // answers but does not boot

  uint8_t id[2];
  if (!readReg(REG_MODEL_ID, id, 2)) return g_beginErr = FW_ERR_NOT_FOUND;
  if (id[0] != 0xEA || id[1] != 0xCC) return g_beginErr = FW_ERR_NOT_FOUND;  // another device

  // Default configuration (as VL53L1X_SensorInit of ST's ULD).
  if (!writeByte(REG_SYSTEM_START, 0x00)) return g_beginErr = FW_ERR_NOT_FOUND;
  uint8_t cfg[CONFIG_LEN];
  for (uint8_t i = 0; i < CONFIG_LEN; i++) cfg[i] = CONFIG[i];
  cfg[0x2E - 0x2D] = 0x01;  // I/O at 2.8 V (VL53L1X-SATEL)
  cfg[0x2F - 0x2D] = 0x01;
  if (!writeTable(REG_CONFIG_FIRST, cfg, CONFIG_LEN)) return g_beginErr = FW_ERR_NOT_FOUND;

  uint8_t v = 0;
  if (!readReg(REG_GPIO_HV_MUX_CTRL, &v, 1)) return g_beginErr = FW_ERR_NOT_FOUND;
  g_pol = (v & 0x10) ? 0 : 1;

  // Init handshake: one measurement, then stop (the ULD does the same).
  if (!writeByte(REG_SYSTEM_START, 0x40)) return g_beginErr = FW_ERR_NOT_FOUND;
  bool got = false;
  t0 = millis();
  while (millis() - t0 < 1000) {
    if (readReg(REG_GPIO_TIO_HV_STATUS, &v, 1) && (v & 1) == g_pol) {
      got = true;
      break;
    }
    delay(1);  // setup-only
  }
  if (!got) return g_beginErr = FW_ERR_TIMEOUT;
  if (!writeByte(REG_SYSTEM_INT_CLEAR, 0x01)) return g_beginErr = FW_ERR_NOT_FOUND;
  if (!writeByte(REG_SYSTEM_START, 0x00)) return g_beginErr = FW_ERR_NOT_FOUND;
  if (!writeByte(0x0008, 0x09) || !writeByte(0x000B, 0x00)) return g_beginErr = FW_ERR_NOT_FOUND;

#if TOF_INF_SHORT_MODE
  // Short distance mode, 33 ms timing budget (values of ST's ULD).
  static const uint8_t WOI[2] = {0x07, 0x05};
  static const uint8_t PHASE[2] = {0x06, 0x06};
  static const uint8_t TB_A[2] = {0x00, 0xD6};
  static const uint8_t TB_B[2] = {0x00, 0x6E};
  bool ok = writeByte(0x004B, 0x14) && writeByte(0x0060, 0x07) && writeByte(0x0063, 0x05) &&
            writeByte(0x0069, 0x38) && writeReg(0x0078, WOI, 2) && writeReg(0x007A, PHASE, 2) &&
            writeReg(0x005E, TB_A, 2) && writeReg(0x0061, TB_B, 2);
  if (!ok) return g_beginErr = FW_ERR_NOT_FOUND;
#endif

  // Time between measurements (as VL53L1X_SetInterMeasurementInMs of the ULD).
  uint8_t osc[2];
  if (!readReg(REG_OSC_CALIB, osc, 2)) return g_beginErr = FW_ERR_NOT_FOUND;
  uint32_t pll = (((uint32_t)osc[0] << 8) | osc[1]) & 0x3FF;
  uint32_t im = pll * INTER_MEAS_MS * 1075UL / 1000UL;  // pll * ms * 1.075
  uint8_t imb[4] = {(uint8_t)(im >> 24), (uint8_t)(im >> 16), (uint8_t)(im >> 8), (uint8_t)im};
  if (!writeReg(REG_INTER_MEAS, imb, 4)) return g_beginErr = FW_ERR_NOT_FOUND;

  if (!writeByte(REG_SYSTEM_START, 0x40)) return g_beginErr = FW_ERR_NOT_FOUND;  // start ranging
  g_tPoll = g_tData = millis();
  g_begun = true;
  return g_beginErr = FW_OK;
}

int tof_inf_update() {
  if (!g_begun)
    return g_beginErr;  // begin() failed (or was not called): keep reporting
                        // why

  uint32_t now = millis();
  if (now - g_tPoll < POLL_MS) return FW_OK;  // not time yet, come back later
  g_tPoll = now;

  uint8_t st;
  if (!readReg(REG_GPIO_TIO_HV_STATUS, &st, 1)) return FW_ERR_NOT_FOUND;
  if ((st & 1) != g_pol) {  // no new measurement yet
    return (now - g_tData > DATA_TIMEOUT_MS) ? FW_ERR_TIMEOUT : FW_OK;
  }

  uint8_t t[RESULT_LEN];
  if (!readReg(REG_RESULT, t, RESULT_LEN)) return FW_ERR_BAD_DATA;      // incomplete read
  if (!writeByte(REG_SYSTEM_INT_CLEAR, 0x01)) return FW_ERR_NOT_FOUND;  // needed after every read
  g_tData = now;

  uint16_t mm = ((uint16_t)t[13] << 8) | t[14];
  if (rangeStatus(t[0]) != 0) return FW_ERR_BAD_DATA;  // sensor says: not a valid measurement
  if (mm > MAX_MM) return FW_ERR_BAD_DATA;             // out of the contract range
  g_last = mm;
  g_ready = true;
  return FW_OK;
}

int tof_inf_get(uint16_t *mm) {
  if (mm == nullptr) return FW_ERR_BAD_DATA;
  if (!g_ready) return FW_ERR_NOT_READY;  // no valid sample yet
  *mm = g_last;
  return FW_OK;
}
