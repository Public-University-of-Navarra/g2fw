#pragma once
#include <fw_errors.h>
#include <stdint.h>

int tof_inf_begin();
int tof_inf_update();
int tof_inf_get(uint16_t *mm);  // distance to the ground, mm (0 to 4000)
