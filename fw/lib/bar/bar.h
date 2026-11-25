// bar: barometer LPS28DFW (contract: fw/README.md, section "bar").
// The same header is used by the mock and by the real driver: it never changes.
#pragma once

#include <fw_errors.h>

struct bar_data_t {
  float hpa;         // air pressure, hPa
  float altitude_m;  // altitude relative to the pressure read in bar_begin(), m
};

int bar_begin();
int bar_update();
int bar_get(bar_data_t *out);
