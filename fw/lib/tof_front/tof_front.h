// tof_front: front distance sensor, obstacles ahead (VL53L8CX, 4x4 zones).
// Contract: fw/README.md, section "tof_front". Same header for the mock and the real driver.
#pragma once

#include <stdint.h>
#include <fw_errors.h>

struct tof_front_data_t {
  uint16_t min_mm;       // closest obstacle over the 16 zones, mm (0 to 4000); 4000 if all zones are clear
  uint16_t zone_mm[16];  // distance of each zone, mm, in the order given by the ST driver;
                         // 4000 = no target in that zone (path clear up to max range)
};

int tof_front_begin();   // may wait up to 5 s while the driver starts (see Rules)
int tof_front_update();
int tof_front_get(tof_front_data_t *out);
