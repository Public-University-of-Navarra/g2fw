// g2fw: integrates every component (contract: fw/README.md).
//
// Each component has its own slot: setupX() runs once in setup() and loopX() runs on every
// loop(). Each pair fills ONLY its own slot, so PRs touch different lines of this file.
#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>
#include <fw_errors.h>

static const char FW_NAME[] = "g2fw";
static const char FW_VERSION[] = "1.0.0";

// ---- imu ----
static void setupImu() {}
static void loopImu() {}

// ---- bar ----
static void setupBar() {}
static void loopBar() {}

// ---- tof_inf ----
#include <tof_inf.h>

static uint32_t tofInfPrintMs = 0;  // last time a tof_inf line was printed
static int tofInfErr = FW_OK;       // last error since the previous print

static void setupTofInf() {
  int err = tof_inf_begin();
  if (err != FW_OK) Serial.printf("[tof_inf] ERR code=%d\r\n", err);
}

static void loopTofInf() {
  int err = tof_inf_update();
  if (err != FW_OK) tofInfErr = err;            // keep it until the next print
  if (millis() - tofInfPrintMs < 1000) return;  // at most one line per second
  tofInfPrintMs = millis();
  uint16_t mm;
  if (tofInfErr != FW_OK) {
    Serial.printf("[tof_inf] ERR code=%d\r\n", tofInfErr);
    tofInfErr = FW_OK;
  } else if (tof_inf_get(&mm) == FW_OK) {
    Serial.printf("[tof_inf] mm=%u\r\n", mm);
  }
}

// ---- tof_front ----
static void setupTofFront() {}
static void loopTofFront() {}

// ---- temp ----
static void setupTemp() {}
static void loopTemp() {}

// ---- camera ----
static void setupCamera() {}
static void loopCamera() {}

void setup() {
  Serial.begin(SERIAL_BAUD);

  // The I2C buses are started here, once. Libraries never call Wire.begin().
  Wire.begin(PIN_I2C_SENS_SDA, PIN_I2C_SENS_SCL, I2C_SENS_FREQ_HZ);
  Wire1.begin(PIN_I2C_TOF_SDA, PIN_I2C_TOF_SCL, I2C_TOF_FREQ_HZ);

  setupImu();
  setupBar();
  setupTofInf();
  setupTofFront();
  setupTemp();
  setupCamera();

  Serial.printf("[fw] name=%s version=%s\n", FW_NAME, FW_VERSION);
  Serial.println("FPF-LAB READY");  // must be the LAST line of setup()
}

void loop() {
  loopImu();
  loopBar();
  loopTofInf();
  loopTofFront();
  loopTemp();
  loopCamera();
}
