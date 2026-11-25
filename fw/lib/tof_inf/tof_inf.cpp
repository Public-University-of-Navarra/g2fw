#include "tof_inf.h"

#include <Arduino.h>

static bool g_ready = false;  // ¿hay ya una muestra válida?
static uint16_t g_last = 0;   // última muestra válida (mm)
static uint32_t g_t = 0;      // última vez que se "midió"
static uint32_t g_n = 0;      // contador de lecturas

int tof_inf_begin() {
  g_t = millis();
  return FW_OK;
}

int tof_inf_update() {
  if (millis() - g_t < 100) return FW_OK;  // 10 veces por segundo, sin esperar
  g_t = millis();
  g_n++;
  if (g_n % 20 == 0) return FW_ERR_TIMEOUT;  // error de prueba cada 20 lecturas
  // Distancia creíble: onda triangular 300..800 mm que cambia poco a poco.
  uint32_t p = g_n % 100;  // 0..99
  g_last = 300 + (p < 50 ? p : 100 - p) * 10;
  g_ready = true;
  return FW_OK;
}

int tof_inf_get(uint16_t *mm) {
  if (mm == nullptr) return FW_ERR_BAD_DATA;
  if (!g_ready) return FW_ERR_NOT_READY;  // aún no hay dato
  *mm = g_last;
  return FW_OK;
}
