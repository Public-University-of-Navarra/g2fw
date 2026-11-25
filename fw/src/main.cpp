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
// ---- imu ----
#include <imu.h>

static void setupImu() {
  int err = imu_begin();
  if (err != FW_OK) {
    Serial.printf("[imu] ERR code=%d\r\n", err);
  }
}

static void loopImu() {
  static uint32_t last_print = 0;

  int err = imu_update();
  if (err != FW_OK) {
    Serial.printf("[imu] ERR code=%d\r\n", err);
  }

  if (millis() - last_print >= 1000) {
    last_print = millis();
    imu_data_t d;
    if (imu_get(&d) == FW_OK) {
      Serial.printf("[imu] ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f\r\n", d.ax, d.ay, d.az,
                    d.gx, d.gy, d.gz);
    }
  }
}
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
#include <tof_front.h>

static uint32_t g_tof_front_last_print_ms = 0;
static int g_tof_front_pending_err = FW_OK;

static void setupTofFront() {
  int err = tof_front_begin();  // real driver: up to 5 s, ~2 s typical (see Rules)
  if (err != FW_OK) {
    Serial.printf("[tof_front] ERR code=%d\r\n", err);
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
    Serial.printf("[tof_front] ERR code=%d\r\n", g_tof_front_pending_err);
    g_tof_front_pending_err = FW_OK;
    return;  // this second: the error line only, like the other components (#69)
  }
  tof_front_data_t data;
  if (tof_front_get(&data) == FW_OK) {
    Serial.printf("[tof_front] mm=%u\r\n", data.min_mm);
  }
}

// ---- temp ----
#include <temp.h>

static uint32_t g_temp_last_print_ms = 0;
static int g_temp_pending_err = FW_OK;

static void setupTemp() {
  int err = temp_begin();
  if (err != FW_OK) {
    Serial.printf("[temp] ERR code=%d\r\n", err);
  }
}

static void loopTemp() {
  int err = temp_update();
  if (err != FW_OK) {
    g_temp_pending_err = err;
  }

  uint32_t now = millis();
  if (now - g_temp_last_print_ms >= 1000) {
    g_temp_last_print_ms = now;
    if (g_temp_pending_err != FW_OK) {
      Serial.printf("[temp] ERR code=%d\r\n", g_temp_pending_err);
      g_temp_pending_err = FW_OK;
    } else {
      float celsius = 0.0f;
      if (temp_get(&celsius) == FW_OK) {
        Serial.printf("[temp] c=%.2f\r\n", celsius);
      }
    }
  }
}

// ---- camera ----
#include <camera.h>

static uint32_t g_camera_print_ms = 0;  // last time a [camera] line was printed
static int g_camera_err = FW_OK;        // last error since the previous print

static void setupCamera() {
  int err = camera_begin();
  if (err != FW_OK) Serial.printf("[camera] ERR code=%d\r\n", err);
}

static void loopCamera() {
  int err = camera_update();
  if (err != FW_OK) g_camera_err = err;  // keep it until the next print

  if (millis() - g_camera_print_ms < 1000) return;  // print once per second, never wait
  g_camera_print_ms = millis();

  if (g_camera_err != FW_OK) {
    Serial.printf("[camera] ERR code=%d\r\n", g_camera_err);
    g_camera_err = FW_OK;
    return;
  }
  camera_data_t d;
  if (camera_get(&d) == FW_OK) Serial.printf("[camera] frames=%lu\r\n", (unsigned long)d.frames);
}

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
