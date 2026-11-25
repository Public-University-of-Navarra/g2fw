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
#include <bar.h>

static uint32_t g_bar_print_ms = 0;  // last time a [bar] line was printed
static int g_bar_err = FW_OK;        // last error since the previous print

static void setupBar() {
  int err = bar_begin();
  if (err != FW_OK) Serial.printf("[bar] ERR code=%d\r\n", err);
}

static void loopBar() {
  int err = bar_update();
  if (err != FW_OK) g_bar_err = err;  // keep it until the next print

  if (millis() - g_bar_print_ms < 1000) return;  // print once per second, never wait
  g_bar_print_ms = millis();

  if (g_bar_err != FW_OK) {
    Serial.printf("[bar] ERR code=%d\r\n", g_bar_err);
    g_bar_err = FW_OK;
    return;
  }
  bar_data_t d;
  if (bar_get(&d) == FW_OK) Serial.printf("[bar] hpa=%.2f alt_m=%.2f\r\n", d.hpa, d.altitude_m);
}

// ---- tof_inf ----
static void setupTofInf() {}
static void loopTofInf() {}

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

  Serial.printf("[fw] name=%s version=%s\r\n", FW_NAME, FW_VERSION);
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
