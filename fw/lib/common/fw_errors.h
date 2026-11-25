// Shared return codes of every component (contract: fw/README.md, "Error codes").
// No C++ exceptions: every function returns one of these values.
#pragma once

enum fw_error {
  FW_OK = 0,             // everything fine
  FW_ERR_NOT_FOUND = 1,  // the device did not answer (wrong address, not connected)
  FW_ERR_TIMEOUT = 2,    // a read started, or a pulse was expected, and nothing came
  FW_ERR_BAD_DATA = 3,   // data out of range, wrong length or wrong checksum
  FW_ERR_NOT_READY = 4,  // called before begin(), or there is no valid sample yet
};
