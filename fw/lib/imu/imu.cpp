#include "imu.h"

#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>
#include <fw_errors.h>
#include <math.h>
#include <string.h>

// ============================================================================
// 1. MAPA DE REGISTROS Y DIRECCIÓN I2C (Datasheet ST LSM6DSV16X)
// ============================================================================
// Dirección I2C de 7 bits cuando el pin SA0/SDO está conectado a VDD (3.3V).
// (Si SA0 estuviera a GND, sería 0x6A).
static constexpr uint8_t LSM6DSV_ADDR = 0x6B;

// Registros internos del sensor que vamos a leer/escribir:
static constexpr uint8_t REG_INT1_CTRL = 0x0D;  // Configura qué eventos disparan el pin físico INT1
static constexpr uint8_t REG_WHO_AM_I = 0x0F;  // Registro fijo de identificación del chip
static constexpr uint8_t WHO_AM_I_EXPECTED = 0x70;  // Valor fijo de fábrica del LSM6DSV16X (0x70)
static constexpr uint8_t REG_CTRL1 =
    0x10;  // Control acelerómetro: modo de potencia y frecuencia (ODR)
static constexpr uint8_t REG_CTRL2 =
    0x11;  // Control giroscopio: modo de potencia y frecuencia (ODR)
static constexpr uint8_t REG_CTRL3 =
    0x12;  // Control general: reset por software, BDU e autoincremento
static constexpr uint8_t REG_CTRL6 = 0x15;  // Fondo de escala (rango máximo) del giroscopio
static constexpr uint8_t REG_CTRL8 = 0x17;  // Fondo de escala (rango máximo) del acelerómetro
static constexpr uint8_t REG_OUTX_L_G =
    0x22;  // Primer byte de datos: empieza Gyro X (L/H), Y, Z y sigue Accel X, Y, Z

// ============================================================================
// 2. FACTORES DE CONVERSIÓN FÍSICA (LSB -> Unidades SI)
// ============================================================================
// El sensor entrega enteros de 16 bits con signo (int16_t: de -32768 a +32767).
// Según el datasheet del LSM6DSV16X:
// - Para ±4g, cada bit menos significativo (1 LSB) equivale a 0.122 mg.
//   Multiplicamos por 0.001 para pasar a 'g' y por 9.80665 para pasar a m/s^2.
static constexpr float ACCEL_SENS_4G = 0.122f * 0.001f * 9.80665f;

// - Para ±500 dps (grados/segundo), cada 1 LSB equivale a 17.50 mdps.
//   Multiplicamos por 0.001 para pasar directamente a deg/s.
static constexpr float GYRO_SENS_500DPS = 17.50f * 0.001f;

// ============================================================================
// 3. UMBRALES DE SEGURIDAD Y TEMPORIZACIÓN
// ============================================================================
// Margen del 5 % sobre el fondo de escala para detectar lecturas corruptas o saturadas.
static constexpr float MAX_ACCEL_MS2 = 4.0f * 9.80665f * 1.05f;
static constexpr float MAX_GYRO_DPS = 500.0f * 1.05f;

// Como configuramos el sensor a 120 Hz, genera una muestra nueva cada 1/120 s ≈ 8.33 ms.
// Si pasan más de 50 ms (~6 muestras perdidas) sin interrupción, damos error de timeout.
static constexpr uint32_t SAMPLE_TIMEOUT_MS = 50;

// ============================================================================
// 4. ESTADO INTERNO DEL DRIVER Y VARIABLES DE INTERRUPCIÓN
// ============================================================================
static bool g_initialized = false;     // true si imu_begin() encontró y configuró el chip
static bool g_ready = false;           // true en cuanto tenemos al menos una muestra válida
static imu_data_t g_last = {};         // Última muestra válida lista para entregar en imu_get()
static uint32_t g_last_sample_ms = 0;  // Marca de tiempo (ms) de la última muestra recibida

// 'volatile' evita que el compilador optimice estas variables pensando que no cambian solas,
// ya que las modifica la rutina de interrupción (ISR) en segundo plano.
static volatile bool g_drdy_flag = false;  // Se pone a true cuando el sensor avisa por INT1
static volatile uint32_t g_drdy_us = 0;    // Instante exacto (us) en que saltó la interrupción

// Rutina de Servicio de Interrupción (ISR):
// - IRAM_ATTR obliga a guardar esta función en la RAM interna del ESP32 (no en Flash)
//   para que pueda ejecutarse al instante aunque la Flash esté ocupada.
// - Nunca hacemos I2C aquí dentro (Wire usa interrupciones y bloquearía el micro).
static void IRAM_ATTR on_imu_int1() {
  g_drdy_us = micros();
  g_drdy_flag = true;
}

// ============================================================================
// 5. FUNCIONES AUXILIARES DE COMUNICACIÓN I2C (Wire)
// ============================================================================
// Escribe 1 byte ('val') en un registro interno ('reg') del sensor.
static bool write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6DSV_ADDR);
  Wire.write(reg);
  Wire.write(val);
  // endTransmission() devuelve 0 si el esclavo respondió con ACK correctamente.
  return (Wire.endTransmission() == 0);
}

// Lee 'len' bytes consecutivos empezando en 'start_reg' y los guarda en 'buf'.
static bool read_regs(uint8_t start_reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(LSM6DSV_ADDR);
  Wire.write(start_reg);
  // 'false' envía un Repeated Start (no suelta el bus I2C) para encadenar la lectura.
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
  g_ready = false;
  g_drdy_flag = false;
  memset(&g_last, 0, sizeof(g_last));

  // Paso A: Comprobar que el sensor está conectado y responde en el bus I2C_SENS
  // (main.cpp ya ejecutó Wire.begin(...) antes de llamar a imu_begin()).
  uint8_t who = 0;
  if (!read_regs(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_EXPECTED) {
    return FW_ERR_NOT_FOUND;
  }

  // Paso B: Reset por software (bit 0 de CTRL3 = 1) para borrar cualquier configuración previa
  if (!write_reg(REG_CTRL3, 0x01)) {
    return FW_ERR_TIMEOUT;
  }
  delay(10);  // setup-only: tiempo que tarda el chip en reiniciar su lógica interna

  // Paso C: Configurar CTRL3 (0x44 = 0b0100_0100):
  // - Bit 6 (BDU = 1): Block Data Update. Impide que el sensor sobrescriba el byte alto (MSB)
  //   mientras estamos leyendo el byte bajo (LSB), evitando mezclar mitades de muestras distintas.
  // - Bit 2 (IF_INC = 1): Auto-incrementa la dirección de registro en cada byte leído en ráfaga.
  if (!write_reg(REG_CTRL3, 0x44)) {
    return FW_ERR_TIMEOUT;
  }

  // Paso D: Rango de medida del acelerómetro en CTRL8 (0x01 -> ±4g)
  if (!write_reg(REG_CTRL8, 0x01)) {
    return FW_ERR_TIMEOUT;
  }

  // Paso E: Rango de medida del giroscopio en CTRL6 (0x02 -> ±500 dps)
  if (!write_reg(REG_CTRL6, 0x02)) {
    return FW_ERR_TIMEOUT;
  }

  // Paso F: Encender acelerómetro en modo High-Performance a 120 Hz (CTRL1 = 0x06)
  if (!write_reg(REG_CTRL1, 0x06)) {
    return FW_ERR_TIMEOUT;
  }

  // Paso G: Encender giroscopio en modo High-Performance a 120 Hz (CTRL2 = 0x06)
  if (!write_reg(REG_CTRL2, 0x06)) {
    return FW_ERR_TIMEOUT;
  }

  // Paso H: Enrutar la señal física "Data Ready" del acelerómetro al pin INT1 (bit 0 = 1).
  // Cada vez que haya una muestra nueva lista, el chip pondrá el pin INT1_IMU a nivel alto (3.3V).
  if (!write_reg(REG_INT1_CTRL, 0x01)) {
    return FW_ERR_TIMEOUT;
  }

  // Paso I: Configurar el pin del ESP32 (definido en board_pins.h) y asociar nuestra ISR
  // al flanco de subida (RISING: paso de 0V a 3.3V).
  pinMode(PIN_IMU_INT1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT1), on_imu_int1, RISING);

  g_last_sample_ms = millis();
  g_initialized = true;
  return FW_OK;
}

// ============================================================================
// 7. ACTUALIZACIÓN NO BLOQUEANTE (Se llama continuamente en loop)
// ============================================================================
int imu_update() {
  // Si imu_begin() falló (por ejemplo, sensor desconectado), no intentamos leer nada.
  if (!g_initialized) {
    return FW_OK;
  }

  // Si aún no ha saltado la interrupción de dato nuevo:
  if (!g_drdy_flag) {
    // Comprobamos si el sensor se ha quedado colgado (lleva demasiado sin avisar)
    if (millis() - g_last_sample_ms > SAMPLE_TIMEOUT_MS) {
      return FW_ERR_TIMEOUT;
    }
    // Si está dentro del tiempo normal entre muestras (~8.3 ms), salimos sin bloquear
    return FW_OK;
  }

  // Bajamos la bandera antes de leer por I2C para no perder el siguiente aviso
  g_drdy_flag = false;

  // Leemos de golpe 12 bytes desde 0x22 (OUTX_L_G) hasta 0x2D (OUTZ_H_A).
  // El mapa interno del LSM6DSV16X coloca primero los 6 bytes del giroscopio
  // y justo a continuación los 6 bytes del acelerómetro, en formato Little-Endian (LSB primero).
  uint8_t raw[12] = {0};
  if (!read_regs(REG_OUTX_L_G, raw, sizeof(raw))) {
    return FW_ERR_TIMEOUT;
  }

  // Reconstruimos los enteros de 16 bits con signo uniendo (MSB << 8) | LSB:
  int16_t raw_gx = static_cast<int16_t>((raw[1] << 8) | raw[0]);
  int16_t raw_gy = static_cast<int16_t>((raw[3] << 8) | raw[2]);
  int16_t raw_gz = static_cast<int16_t>((raw[5] << 8) | raw[4]);

  int16_t raw_ax = static_cast<int16_t>((raw[7] << 8) | raw[6]);
  int16_t raw_ay = static_cast<int16_t>((raw[9] << 8) | raw[8]);
  int16_t raw_az = static_cast<int16_t>((raw[11] << 8) | raw[10]);

  // Convertimos a unidades físicas reales (deg/s y m/s^2)
  imu_data_t sample;
  sample.gx = raw_gx * GYRO_SENS_500DPS;
  sample.gy = raw_gy * GYRO_SENS_500DPS;
  sample.gz = raw_gz * GYRO_SENS_500DPS;

  sample.ax = raw_ax * ACCEL_SENS_4G;
  sample.ay = raw_ay * ACCEL_SENS_4G;
  sample.az = raw_az * ACCEL_SENS_4G;

  // Validación de seguridad: descartamos la muestra si algún eje es NaN/Infinito
  // o supera el rango físico configurado (±4g / ±500 dps).
  if (!isfinite(sample.ax) || !isfinite(sample.ay) || !isfinite(sample.az) ||
      !isfinite(sample.gx) || !isfinite(sample.gy) || !isfinite(sample.gz) ||
      fabsf(sample.ax) > MAX_ACCEL_MS2 || fabsf(sample.ay) > MAX_ACCEL_MS2 ||
      fabsf(sample.az) > MAX_ACCEL_MS2 || fabsf(sample.gx) > MAX_GYRO_DPS ||
      fabsf(sample.gy) > MAX_GYRO_DPS || fabsf(sample.gz) > MAX_GYRO_DPS) {
    return FW_ERR_BAD_DATA;
  }

  // Guardamos la muestra ya validada y actualizamos el reloj de última recepción
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
  // Según el contrato, si aún no hay ninguna muestra válida, no tocamos *out
  if (!g_ready) {
    return FW_ERR_NOT_READY;
  }
  *out = g_last;
  return FW_OK;
}