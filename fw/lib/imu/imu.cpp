#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>
#include <fw_errors.h>
#include <math.h>

// ============================================================================
// 1. MAPA DE REGISTROS Y DIRECCIÓN I2C (Datasheet ST LSM6DSV16X)
// ============================================================================
// Dirección I2C de 7 bits cuando el pin SA0/SDO está conectado a VDD (3.3V).
// (Si SA0 estuviera a GND, sería 0x6A).
static constexpr uint8_t LSM6DSV_ADDR = 0x6B;

// Configura qué eventos disparan el pin físico INT1
static constexpr uint8_t REG_INT1_CTRL = 0x0D;
// Registro fijo de identificación del chip
static constexpr uint8_t REG_WHO_AM_I = 0x0F;
// Valor fijo de fábrica del LSM6DSV16X (0x70)
static constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;
// Control acelerómetro: modo de potencia y frecuencia (ODR)
static constexpr uint8_t REG_CTRL1 = 0x10;
// Control giroscopio: modo de potencia y frecuencia (ODR)
static constexpr uint8_t REG_CTRL2 = 0x11;
// Control general: reset por software, BDU e autoincremento
static constexpr uint8_t REG_CTRL3 = 0x12;
// Fondo de escala (rango máximo) del giroscopio
static constexpr uint8_t REG_CTRL6 = 0x15;
// Fondo de escala (rango máximo) del acelerómetro
static constexpr uint8_t REG_CTRL8 = 0x17;
// Primer byte de datos: empieza Gyro X (L/H), Y, Z y sigue Accel X, Y, Z
static constexpr uint8_t REG_OUTX_L_G = 0x22;

// ============================================================================
// 2. FACTORES DE CONVERSIÓN FÍSICA (LSB -> Unidades SI)
// ============================================================================
// Para ±4g, 1 LSB = 0.122 mg -> m/s^2
static constexpr float ACCEL_SENS_4G = 0.122f * 0.001f * 9.80665f;
// Para ±500 dps, 1 LSB = 17.50 mdps -> deg/s
static constexpr float GYRO_SENS_500DPS = 17.50f * 0.001f;

// ============================================================================
// 3. UMBRALES DE SEGURIDAD Y TEMPORIZACIÓN
// ============================================================================
static constexpr float MAX_ACCEL_MS2 = 4.0f * 9.80665f * 1.05f;
static constexpr float MAX_GYRO_DPS = 500.0f * 1.05f;
static constexpr uint32_t SAMPLE_TIMEOUT_MS = 50;

// ============================================================================
// 4. ESTADO INTERNO DEL DRIVER Y VARIABLES DE INTERRUPCIÓN
// ============================================================================
static bool g_initialized = false;
static int g_begin_err = FW_ERR_NOT_READY;
static bool g_ready = false;
static imu_data_t g_last = {};
static uint32_t g_last_sample_ms = 0;

static volatile bool g_drdy_flag = false;
static volatile uint32_t g_drdy_us = 0;

static void IRAM_ATTR on_imu_int1() {
  g_drdy_us = micros();
  g_drdy_flag = true;
}

// ============================================================================
// 5. FUNCIONES AUXILIARES DE COMUNICACIÓN I2C (Wire)
// ============================================================================
static bool write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6DSV_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool read_regs(uint8_t start_reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(LSM6DSV_ADDR);
  Wire.write(start_reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received = Wire.requestFrom(LSM6DSV_ADDR, static_cast<uint8_t>(len));
  if (received != len) {
    return false;
  }

  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

// ============================================================================
// 6. INICIALIZACIÓN DEL SENSOR (Se llama una sola vez en setup)
// ============================================================================
int imu_begin() {
  g_initialized = false;
  g_begin_err = FW_ERR_NOT_READY;
  g_ready = false;
  g_drdy_flag = false;
  g_last = {};

  uint8_t who = 0;
  if (!read_regs(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_EXPECTED) {
    g_begin_err = FW_ERR_NOT_FOUND;
    return g_begin_err;
  }

  // Software reset
  if (!write_reg(REG_CTRL3, 0x01)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }
  delay(10);  // setup-only

  // CTRL3: BDU=1 (0x40) + IF_INC=1 (0x04)
  if (!write_reg(REG_CTRL3, 0x44)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }

  // CTRL8: Accel Full-Scale = ±4g (0x01)
  if (!write_reg(REG_CTRL8, 0x01)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }

  // CTRL6: Gyro Full-Scale = ±500 dps (0x02)
  if (!write_reg(REG_CTRL6, 0x02)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }

  // CTRL1: Accel High-Performance + ODR 120 Hz (0x06)
  if (!write_reg(REG_CTRL1, 0x06)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }

  // CTRL2: Gyro High-Performance + ODR 120 Hz (0x06)
  if (!write_reg(REG_CTRL2, 0x06)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }

  // INT1_CTRL: Data-Ready de acelerómetro en INT1 (0x01)
  if (!write_reg(REG_INT1_CTRL, 0x01)) {
    g_begin_err = FW_ERR_TIMEOUT;
    return g_begin_err;
  }

  pinMode(PIN_IMU_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT1), on_imu_int1, RISING);

  g_last_sample_ms = millis();
  g_initialized = true;
  g_begin_err = FW_OK;
  return FW_OK;
}

// ============================================================================
// 7. ACTUALIZACIÓN NO BLOQUEANTE (Se llama continuamente en loop)
// ============================================================================
int imu_update() {
  if (!g_initialized) {
    return g_begin_err;
  }

  // Comprobamos tanto la bandera de interrupción como el nivel físico del pin INT1
  // (enclavado en HIGH hasta que se leen los registros de salida). Si salta timeout,
  // forzamos una lectura de recuperación para limpiar el latch.
  if (!g_drdy_flag && digitalRead(PIN_IMU_INT1) == LOW) {
    return (millis() - g_last_sample_ms > SAMPLE_TIMEOUT_MS) ? FW_ERR_TIMEOUT : FW_OK;
  }

  g_drdy_flag = false;

  uint8_t raw[12] = {0};
  if (!read_regs(REG_OUTX_L_G, raw, sizeof(raw))) {
    return FW_ERR_TIMEOUT;
  }

  int16_t raw_gx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
  int16_t raw_gy = static_cast<int16_t>((raw[3] << 8) | raw[2]);
  int16_t raw_gz = static_cast<int16_t>((raw[5] << 8) | raw[4]);

  int16_t raw_ax = static_cast<int16_t>((raw[7] << 8) | raw[6]);
  int16_t raw_ay = static_cast<int16_t>((raw[9] << 8) | raw[8]);
  int16_t raw_az = static_cast<int16_t>((raw[11] << 8) | raw[10]);

  imu_data_t sample;
  sample.gx = raw_gx * GYRO_SENS_500DPS;
  sample.gy = raw_gy * GYRO_SENS_500DPS;
  sample.gz = raw_gz * GYRO_SENS_500DPS;

  sample.ax = raw_ax * ACCEL_SENS_4G;
  sample.ay = raw_ay * ACCEL_SENS_4G;
  sample.az = raw_az * ACCEL_SENS_4G;

  if (!isfinite(sample.ax) || !isfinite(sample.ay) || !isfinite(sample.az) ||
      !isfinite(sample.gx) || !isfinite(sample.gy) || !isfinite(sample.gz) ||
      fabsf(sample.ax) > MAX_ACCEL_MS2 || fabsf(sample.ay) > MAX_ACCEL_MS2 ||
      fabsf(sample.az) > MAX_ACCEL_MS2 || fabsf(sample.gx) > MAX_GYRO_DPS ||
      fabsf(sample.gy) > MAX_GYRO_DPS || fabsf(sample.gz) > MAX_GYRO_DPS) {
    return FW_ERR_BAD_DATA;
  }

  g_last = sample;
  g_last_sample_ms = millis();
  g_ready = true;
  return FW_OK;
}

// ============================================================================
// 8. LECTURA DE LA ÚLTIMA MUESTRA VÁLIDA (Getter)
// ============================================================================
int imu_get(imu_data_t* out) {
  if (out == nullptr) {
    return FW_ERR_BAD_DATA;
  }
  if (!g_ready) {
    return FW_ERR_NOT_READY;
  }
  *out = g_last;
  return FW_OK;
}