// ESP32 pin map of the drone sensor board (contract: fw/README.md, "Pin mapping").
//
// Source: the board sent to fabrication (rev B, 19/11/2026). Schematic and PCB give the same
// pins. The rev A pins (first routing) are no longer valid.
// This is the ONLY place where pin numbers are written. Numbers are GPIO, not header pins.
// Module: ESP32-WROOM-32E without PSRAM. IO16/IO17 are free only on WROOM modules (a WROVER
// uses them for its PSRAM): do not swap the module without moving INT_TOF_INF and INT_ALT.
#pragma once

#include <stdint.h>

// I2C_SENS bus -> Wire: VL53L1X, MCP9808, LPS28DFW, LSM6DSV16X.
constexpr int PIN_I2C_SENS_SDA = 21;
constexpr int PIN_I2C_SENS_SCL = 22;
constexpr uint32_t I2C_SENS_FREQ_HZ = 400000;

// I2C_TOF bus -> Wire1: only the VL53L8CX (same address 0x29 as the VL53L1X).
constexpr int PIN_I2C_TOF_SDA = 27;
constexpr int PIN_I2C_TOF_SCL = 26;
constexpr uint32_t I2C_TOF_FREQ_HZ = 400000;

// imu: LSM6DSV16X.
constexpr int PIN_IMU_INT1 = 19;
constexpr int PIN_IMU_INT2 = 18;  // spare

// bar: LPS28DFW.
constexpr int PIN_ALT_INT = 17;

// tof_inf: VL53L1X (pull-up of INT on the SATEL board).
constexpr int PIN_TOF_INF_XSHUT = 4;  // LOW = sensor off
constexpr int PIN_TOF_INF_INT = 16;

// tof_front: VL53L8CX.
constexpr int PIN_TOF_FRONT_LPN = 13;
// Strapping pin (MTDI): HIGH at boot sets the flash to 1.8 V and the ESP32 does not start.
// The SATEL 100 k pull-down keeps it LOW. Never add a pull-up.
constexpr int PIN_TOF_FRONT_PWR_EN = 12;
constexpr int PIN_TOF_FRONT_INT = 34;   // input only, no internal pull-up
constexpr int PIN_TOF_FRONT_SYNC = 14;  // pulses during boot: keep as INPUT until used

// camera: pulse from the DSP at the start of every image.
constexpr int PIN_DSP_SYNC = 33;

// Status LED D701 (strapping pin, the LED to GND keeps it LOW at boot).
constexpr int PIN_LED_STATUS = 2;

// Reserved links (not used in version 1.0.0).
constexpr int PIN_DSP_UART_TX = 32;  // DSP_LINK_A
constexpr int PIN_DSP_UART_RX = 35;  // DSP_LINK_B, input only
constexpr int PIN_FC_UART_TX = 25;   // Pixhawk TELEM
constexpr int PIN_FC_UART_RX = 39;   // input only

// I2C addresses, 7 bits.
constexpr uint8_t ADDR_MCP9808 = 0x18;
constexpr uint8_t ADDR_VL53L1X = 0x29;
constexpr uint8_t ADDR_LPS28DFW = 0x5C;
constexpr uint8_t ADDR_LSM6DSV16X = 0x6B;
constexpr uint8_t ADDR_VL53L8CX = 0x29;  // on I2C_TOF

constexpr uint32_t SERIAL_BAUD = 115200;
