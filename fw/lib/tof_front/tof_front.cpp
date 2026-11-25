// tof_front: REAL driver for the VL53L8CX on the SATEL-VL53L8 board (I2C_TOF bus, Wire1, 0x29).
// Contract: fw/README.md, section "tof_front". Header: tof_front.h (the same one as the mock).
//
// Adapted from test 05 of the validation firmware (week of 19/11/2026), with the rev B pins
// from fw/lib/common/board_pins.h: LPn = IO13, PWR_EN = IO12, INT = IO34, SYNC = IO14.
// Library: stm32duino/STM32duino VL53L8CX 2.1.0 (lib_deps in fw/platformio.ini).
//
// IMPORTANT, ST library behaviour (checked in its source, vl53l8cx.h, IO_Read/IO_Write):
// if the sensor stops answering (NAK), the library retries FOREVER and the whole firmware
// freezes. That is why we check the ACK at 0x29 ourselves before every call to the library.
#include "tof_front.h"

#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>
#include <vl53l8cx.h>

namespace {

constexpr uint8_t ZONES = 16;                // 4x4
constexpr uint8_t FREQUENCY_HZ = 60;         // contract: 60 Hz in 4x4
constexpr uint16_t NO_TARGET_MM = 4000;      // contract: no target = 4000
constexpr uint32_t START_TIMEOUT_MS = 5000;  // contract: start-up bounded to 5 s
constexpr uint32_t POWER_UP_MS = 200;        // max wait for the ACK after PWR_EN = 1
constexpr uint32_t FRAME_TIMEOUT_MS = 500;   // no frame for 0.5 s (30 frames) -> FW_ERR_TIMEOUT

// ST target status: only 5 and 9 are valid measurements (UM3109).
constexpr uint8_t ST_STATUS_VALID = 5;
constexpr uint8_t ST_STATUS_VALID_LARGE_PULSE = 9;

// i2c_rst_pin = -1: SPI_I2C_N is tied to GND on the SATEL, the ESP32 does not drive it.
VL53L8CX sensor(&Wire1, PIN_TOF_FRONT_LPN, -1);

// Global, not on the stack: it is large and loop() has only 8 KB of stack.
VL53L8CX_ResultsData results;

bool started = false;
int begin_err = FW_ERR_NOT_READY;  // result of begin(), repeated by update() if it failed
bool has_sample = false;
uint32_t last_frame_ms = 0;
tof_front_data_t last_valid = {};

// Written by the interrupt routine (contract Rules: IRAM_ATTR, volatile, critical section).
portMUX_TYPE int_mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool int_flag = false;
volatile uint32_t int_us = 0;  // micros() of the last INT: timestamp of the frame

void IRAM_ATTR on_int() {
  portENTER_CRITICAL_ISR(&int_mux);
  int_us = micros();
  int_flag = true;
  portEXIT_CRITICAL_ISR(&int_mux);
}

bool sensor_acks() {
  Wire1.beginTransmission(ADDR_VL53L8CX);
  return Wire1.endTransmission() == 0;
}

// Only for begin(): waits until the sensor answers or the time runs out.
bool wait_for_ack(uint32_t timeout_ms) {
  uint32_t t0 = millis();
  do {
    if (sensor_acks()) return true;
    delay(2);  // setup-only
  } while (millis() - t0 < timeout_ms);
  return false;
}

// ST status codes (vl53l8cx_api.h) -> contract error codes.
int map_st_status(uint8_t st) {
  if (st == VL53L8CX_STATUS_OK) return FW_OK;
  if (st == VL53L8CX_STATUS_TIMEOUT_ERROR) return FW_ERR_TIMEOUT;
  return FW_ERR_BAD_DATA;  // corrupted frame, FW checksum, invalid parameter...
}

// Copies the ST results into the contract format, validating every zone.
void fill_sample(tof_front_data_t &frame) {
  frame.min_mm = NO_TARGET_MM;
  for (uint8_t zone = 0; zone < ZONES; zone++) {
    uint16_t idx = (uint16_t)VL53L8CX_NB_TARGET_PER_ZONE * zone;
    uint8_t status = results.target_status[idx];
    bool valid = results.nb_target_detected[zone] > 0 &&
                 (status == ST_STATUS_VALID || status == ST_STATUS_VALID_LARGE_PULSE);
    uint16_t d = NO_TARGET_MM;
    if (valid) {
      int16_t raw = results.distance_mm[idx];
      if (raw < 0) raw = 0;  // validate range: 0 to 4000
      d = (raw > NO_TARGET_MM) ? NO_TARGET_MM : (uint16_t)raw;
    }
    frame.zone_mm[zone] = d;
    if (d < frame.min_mm) frame.min_mm = d;
  }
}

}  // namespace

static int begin_impl();

int tof_front_begin() {
  begin_err = begin_impl();
  started = (begin_err == FW_OK);
  return begin_err;
}

static int begin_impl() {
  started = false;
  has_sample = false;
  uint32_t t0 = millis();

  // SYNC is not used (free-running mode). IO14 gives pulses at boot: keep it as an input.
  pinMode(PIN_TOF_FRONT_SYNC, INPUT);
  pinMode(PIN_TOF_FRONT_INT, INPUT);  // IO34: input only, the pull-up is on the SATEL board

  // Power cycle through PWR_EN (IO12): also recovers a sensor left on by an ESP32 reset.
  pinMode(PIN_TOF_FRONT_LPN, OUTPUT);
  digitalWrite(PIN_TOF_FRONT_LPN, HIGH);  // I2C of the sensor enabled
  pinMode(PIN_TOF_FRONT_PWR_EN, OUTPUT);
  digitalWrite(PIN_TOF_FRONT_PWR_EN, LOW);
  delay(10);  // setup-only: regulators of the SATEL fully off
  digitalWrite(PIN_TOF_FRONT_PWR_EN, HIGH);

  // Without this check, the ST init would retry forever if the sensor is missing (e.g. Wokwi).
  if (!wait_for_ack(POWER_UP_MS)) return FW_ERR_NOT_FOUND;

  sensor.begin();  // pulses LPn (0 for 10 ms, then 1); it does not start Wire1 (main.cpp does)
  if (!wait_for_ack(POWER_UP_MS)) return FW_ERR_NOT_FOUND;

  // Uploads the ~84 KB firmware of the sensor (~2 s at 400 kHz). The only long wait allowed.
  uint8_t st = sensor.init();
  if (st != VL53L8CX_STATUS_OK) return map_st_status(st);
  if (millis() - t0 > START_TIMEOUT_MS) return FW_ERR_TIMEOUT;

  // The frequency must be set AFTER the resolution (ST UM3109).
  st = sensor.set_resolution(VL53L8CX_RESOLUTION_4X4);
  if (st != VL53L8CX_STATUS_OK) return map_st_status(st);
  st = sensor.set_ranging_frequency_hz(FREQUENCY_HZ);
  if (st != VL53L8CX_STATUS_OK) return map_st_status(st);

  int_flag = false;
  attachInterrupt(digitalPinToInterrupt(PIN_TOF_FRONT_INT), on_int, FALLING);  // INT active low

  st = sensor.start_ranging();
  if (st != VL53L8CX_STATUS_OK) return map_st_status(st);
  if (millis() - t0 > START_TIMEOUT_MS) return FW_ERR_TIMEOUT;

  last_frame_ms = millis();
  return FW_OK;
}

int tof_front_update() {
  // If begin() failed (e.g. no sensor, as in Wokwi), keep reporting why: ERR code=1, not 4.
  if (!started) return begin_err;

  // Read the flag and the timestamp together (contract Rules: critical section).
  bool new_frame;
  portENTER_CRITICAL(&int_mux);
  new_frame = int_flag;
  int_flag = false;
  portEXIT_CRITICAL(&int_mux);

  if (!new_frame) {
    // No INT: nothing to do, unless the sensor has stopped sending frames.
    if (millis() - last_frame_ms > FRAME_TIMEOUT_MS) {
      last_frame_ms = millis();  // report it at most once every FRAME_TIMEOUT_MS
      return FW_ERR_TIMEOUT;
    }
    return FW_OK;
  }

  // The ST library would hang forever on a NAK: check the sensor is still there first.
  if (!sensor_acks()) return FW_ERR_NOT_FOUND;

  uint8_t ready = 0;
  uint8_t st = sensor.check_data_ready(&ready);
  if (st != VL53L8CX_STATUS_OK) return map_st_status(st);
  if (!ready) return FW_OK;  // spurious edge on INT: wait for the next one

  st = sensor.get_ranging_data(&results);
  if (st != VL53L8CX_STATUS_OK) return map_st_status(st);  // corrupted frame: keep last valid

  tof_front_data_t frame;
  fill_sample(frame);
  last_valid = frame;
  has_sample = true;
  last_frame_ms = millis();
  return FW_OK;
}

int tof_front_get(tof_front_data_t *out) {
  if (out == nullptr) return FW_ERR_BAD_DATA;
  if (!has_sample) return FW_ERR_NOT_READY;  // *out untouched
  *out = last_valid;
  return FW_OK;
}
