#include "imu.h"
#include <Arduino.h>
#include <fw_errors.h>
#include <math.h>

static bool g_ready = false;    // ¿hay ya una muestra válida?
static imu_data_t g_last = {0}; // última muestra válida
static uint32_t g_t = 0;        // última vez que se "midió"
static uint32_t g_n = 0;        // contador de lecturas

int imu_begin()
{
    g_t = millis();
    g_ready = false;
    g_n = 0;
    return FW_OK;
}

int imu_update()
{
    if (millis() - g_t < 100)
        return FW_OK; // 10 Hz en mock (120 Hz en el LSM6DSV16X real)
    g_t = millis();
    g_n++;
    if (g_n % 20 == 0)
        return FW_ERR_TIMEOUT; // error de prueba cada 20 lecturas

    // Valores creíbles en reposo con ligera oscilación sinusoidal para verificar cambios en el log
    float phase = g_n * 0.1f;
    g_last.ax = 0.02f * sinf(phase);
    g_last.ay = -0.01f * cosf(phase);
    g_last.az = 9.81f + 0.03f * sinf(phase); // Gravedad en Z (m/s^2)
    g_last.gx = 0.10f * cosf(phase);
    g_last.gy = -0.20f * sinf(phase);
    g_last.gz = 0.05f * sinf(phase * 0.5f);

    g_ready = true;
    return FW_OK;
}

int imu_get(imu_data_t *out)
{
    if (out == nullptr)
        return FW_ERR_BAD_DATA;
    if (!g_ready)
    {
        *out = {0}; // Pone a 0 por seguridad si se lee antes de tiempo
        return FW_ERR_NOT_READY;
    }
    *out = g_last;
    return FW_OK;
}