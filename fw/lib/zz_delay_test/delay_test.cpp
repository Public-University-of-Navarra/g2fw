// TEMPORARY test for #31 (CI): never merged. A delay() without the "setup-only" mark must make
// the lint job fail.
#include <Arduino.h>

void delay_test_begin() { delay(5); }
