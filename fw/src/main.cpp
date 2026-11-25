// g2fw: integrates every component (contract: fw/README.md).
//
// Each component has its own slot: setupX() runs once in setup() and loopX() runs on every
// loop(). Each pair fills ONLY its own slot, so PRs touch different lines of this file.
#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>
#include <fw_errors.h>
#include <imu.h>

static const char FW_NAME[] = "g2fw";
static const char FW_VERSION[] = "1.0.0";

// ---- imu ----
static void setupImu()
{
  int err = imu_begin();
  if (err != FW_OK)
  {
    Serial.printf("[imu] ERR code=%d\r\n", err);
  }
}

static void loopImu()
{
  static uint32_t last_print = 0;

  int err = imu_update();
  if (err != FW_OK)
  {
    Serial.printf("[imu] ERR code=%d\r\n", err);
  }

  if (millis() - last_print >= 1000)
  {
    last_print = millis();
    imu_data_t d;
    if (imu_get(&d) == FW_OK)
    {
      Serial.printf("[imu] ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f\r\n",
                    d.ax, d.ay, d.az, d.gx, d.gy, d.gz);
    }
  }
}

// ---- bar ----
static void setupBar() {}
static void loopBar() {}

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

void setup()
{
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
  Serial.println("FPF-LAB READY"); // must be the LAST line of setup()
}

void loop()
{
  loopImu();
  loopBar();
  loopTofInf();
  loopTofFront();
  loopTemp();
  loopCamera();
}
