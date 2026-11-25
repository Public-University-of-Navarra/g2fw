#pragma once

#include <fw_errors.h>

int temp_begin();
int temp_update();
int temp_get(float* celsius);  // temperature, degC