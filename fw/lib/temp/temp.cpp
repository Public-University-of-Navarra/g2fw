#include "temp.h"

#include <Adafruit_MCP9808.h>
#include <Arduino.h>
#include <Wire.h>
#include <board_pins.h>
#include <math.h>

static constexpr uint32_t SAMPLE_PERIOD_MS = 300;  // > 250 ms de conversión a 0,0625 °C
static constexpr float MCP9808_MIN_C = -40.0f;     // límite inferior datasheet MCP9808
static constexpr float MCP9808_MAX_C = 125.0f;     // límite superior datasheet MCP9808

static Adafruit_MCP9808 g_mcp;
static int g_begin_err = FW_ERR_NOT_FOUND;
static bool g_ready = false;
static float g_last = 0.0f;
static uint32_t g_last_sample_ms = 0;

int temp_begin() {
  g_begin_err = FW_ERR_NOT_FOUND;
  g_ready = false;
  g_last = 0.0f;
  g_last_sample_ms = 0;

  if (!g_mcp.begin(ADDR_MCP9808, &Wire)) {
    return g_begin_err;
  }

  g_mcp.setResolution(3);  // 0,0625 °C (250 ms por conversión)
  g_begin_err = FW_OK;
  return FW_OK;
}

int temp_update() {
  if (g_begin_err != FW_OK) {
    return g_begin_err;  // no intenta leer si el sensor no respondió en temp_begin()
  }

  uint32_t now = millis();
  if (g_last_sample_ms != 0 && (now - g_last_sample_ms < SAMPLE_PERIOD_MS)) {
    return FW_OK;
  }
  g_last_sample_ms = now;

  float c = g_mcp.readTempC();
  if (isnan(c)) {
    return FW_ERR_TIMEOUT;  // empezó a contestar pero falló/se desconectó la lectura
  }
  if (c < MCP9808_MIN_C || c > MCP9808_MAX_C) {
    return FW_ERR_BAD_DATA;  // fuera del rango físico del sensor
  }

  g_last = c;
  g_ready = true;
  return FW_OK;
}

int temp_get(float* celsius) {
  if (celsius == nullptr) return FW_ERR_BAD_DATA;
  if (!g_ready) return FW_ERR_NOT_READY;
  *celsius = g_last;
  return FW_OK;
}