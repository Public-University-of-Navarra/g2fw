// camera: images of the camera, seen through the DSP (contract: fw/README.md, section "camera").
// The camera is wired to the DSP, not to the ESP32: the ESP32 only counts the DSP_SYNC pulses.
// The same header is used by the mock and by the real driver: it never changes.
#pragma once

#include <fw_errors.h>
#include <stdint.h>

struct camera_data_t {
  uint32_t frames;        // DSP_SYNC pulses (images) since boot
  uint32_t last_sync_us;  // micros() of the last pulse
};

int camera_begin();
int camera_update();  // FW_ERR_TIMEOUT if no pulse arrived in the last 1 s
int camera_get(camera_data_t *out);
