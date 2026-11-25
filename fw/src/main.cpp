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
static void setupTofInf() {}
static void loopTofInf() {}

// ---- tof_front ----
#include <tof_front.h>

static uint32_t g_tof_front_last_print_ms = 0;
static int g_tof_front_pending_err = FW_OK;

static void setupTofFront() {
  int err = tof_front_begin();  // may take ~2 s with the real driver (contract, Rules)
  if (err != FW_OK) {
    Serial.printf("[tof_front] ERR code=%d\n", err);
  }
}

static void loopTofFront() {
  int err = tof_front_update();
  if (err != FW_OK) {
    g_tof_front_pending_err = err;  // remember it, print at most once per second
  }

  uint32_t now = millis();
  if (now - g_tof_front_last_print_ms < 1000) return;
  g_tof_front_last_print_ms = now;

  if (g_tof_front_pending_err != FW_OK) {
    Serial.printf("[tof_front] ERR code=%d\n", g_tof_front_pending_err);
    g_tof_front_pending_err = FW_OK;
  }
  tof_front_data_t data;
  if (tof_front_get(&data) == FW_OK) {
    Serial.printf("[tof_front] mm=%u\n", data.min_mm);
  }
}

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
