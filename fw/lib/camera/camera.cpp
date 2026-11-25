// REAL driver of the camera link: counts the DSP_SYNC pulses (one per image) on IO33.
//
// The DSP sends a pulse at the start of every image. An interrupt routine counts the pulses and
// saves micros() of the last one; camera_update() copies both values, with interrupts disabled,
// and reports FW_ERR_TIMEOUT if no pulse arrived in the last 1 s (contract, "camera").
// With no DSP connected (class, Wokwi) the pin stays LOW and "[camera] ERR code=2" is expected.
#include "camera.h"

#include <Arduino.h>
#include <board_pins.h>

static const uint32_t kTimeoutUs = 1000000;  // contract: error if no pulse in the last 1 s
static const uint32_t kMinPeriodUs = 10000;  // reject glitches: PAL images come every 40 ms

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_isr_frames = 0;   // written only by the interrupt routine
static volatile uint32_t g_isr_last_us = 0;  // micros() of the last valid pulse

static bool g_started = false;
static uint32_t g_begin_us = 0;
static camera_data_t g_last = {0, 0};

// Interrupt routine: only saves the time and counts (contract, Rules). Runs from IRAM.
static void IRAM_ATTR onDspSync() {
  uint32_t now_us = micros();
  portENTER_CRITICAL_ISR(&g_mux);
  if (g_isr_frames == 0 || now_us - g_isr_last_us >= kMinPeriodUs) {  // validate: no glitches
    g_isr_frames = g_isr_frames + 1;  // not "++": deprecated on volatile in C++20
    g_isr_last_us = now_us;
  }
  portEXIT_CRITICAL_ISR(&g_mux);
}

int camera_begin() {
  // Pull-down: with no DSP connected the line stays LOW and no false pulse is counted.
  pinMode(PIN_DSP_SYNC, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_DSP_SYNC), onDspSync, RISING);
  g_begin_us = micros();
  g_started = true;
  return FW_OK;
}

int camera_update() {
  if (!g_started) return FW_ERR_NOT_READY;

  // Copy the two values written by the interrupt together, so they belong to the same pulse.
  portENTER_CRITICAL(&g_mux);
  g_last.frames = g_isr_frames;
  g_last.last_sync_us = g_isr_last_us;
  portEXIT_CRITICAL(&g_mux);

  uint32_t ref_us = (g_last.frames > 0) ? g_last.last_sync_us : g_begin_us;  // last pulse, or boot
  if (micros() - ref_us > kTimeoutUs) return FW_ERR_TIMEOUT;  // no pulse in the last second
  return FW_OK;
}

int camera_get(camera_data_t *out) {
  if (out == nullptr) return FW_ERR_BAD_DATA;
  if (g_last.frames == 0) return FW_ERR_NOT_READY;  // no image yet
  *out = g_last;
  return FW_OK;
}
