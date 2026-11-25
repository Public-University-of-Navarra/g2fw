// TEMPORARY test for #31 (CI): never merged. With the "setup-only" mark, the same delay() must
// pass the lint job.
#include <Arduino.h>

void delay_test_begin() {
  delay(5);  // setup-only
}
