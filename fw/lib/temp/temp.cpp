#include "temp.h"

#include <Arduino.h>

static bool g_ready = false;  // ¿hay ya una muestra válida?
static float g_last = 0.0f;   // última muestra válida (°C)
static uint32_t g_t = 0;      // última vez que se "midió"
static uint32_t g_n = 0;      // contador de lecturas
static float g_sim = 21.50f;  // temperatura simulada inicial
static float g_step = 0.05f;  // variación suave por lectura

int temp_begin() {
  g_ready = false;
  g_last = 0.0f;
  g_t = millis();
  g_n = 0;
  g_sim = 21.50f;
  g_step = 0.05f;
  return FW_OK;
}

int temp_update() {
  if (millis() - g_t < 1000) return FW_OK;  // 1 Hz según contrato, sin esperar
  g_t = millis();
  g_n++;
  if (g_n % 20 == 0) return FW_ERR_TIMEOUT;  // error de prueba cada 20 lecturas

  g_sim += g_step;
  if (g_sim >= 22.50f || g_sim <= 21.00f) {
    g_step = -g_step;
  }

  g_last = g_sim;
  g_ready = true;
  return FW_OK;
}

int temp_get(float* celsius) {
  if (celsius == nullptr) return FW_ERR_BAD_DATA;
  if (!g_ready) return FW_ERR_NOT_READY;  // aún no hay dato
  *celsius = g_last;
  return FW_OK;
}