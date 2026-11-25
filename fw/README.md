# g2fw: firmware contract

This file is the **single source of truth** for the firmware. If the code and this README
disagree, the code is wrong, or this README is changed first in its own PR.

- **Board:** ESP32-DevKitC-32E, the module on our drone PCB (`board = esp32dev`). The DOIT ESP32
  DEVKIT V1 used on the breadboard has the same chip and GPIO numbers.
- **PlatformIO env:** `esp32` (`pio run -e esp32`). CI builds it in the job `build (esp32)`.
- **Serial port (UART0, USB of the DevKit):** 115200 baud. This is the output the tests read.
- **Firmware version:** `1.0.0`.

## Components

Every component lives in `fw/lib/<name>/` (`<name>.h` + `<name>.cpp`) and follows the same pattern:

```cpp
#include <fw_errors.h>

int <name>_begin();   // call once in setup(). Returns FW_OK or an error code.
int <name>_update();  // call on every loop(). Never waits. Returns FW_OK if there was nothing
                      // to do or the read worked, or an error code if a read failed.
// getters: return the LAST VALID value. They never touch the hardware.
```

Before the first valid sample the getters return `0`. The I2C buses are started once in
`main.cpp` (see *Pin mapping*); a library never calls `Wire.begin()` or `Wire1.begin()`.

| Component   | Device                | Bus            | Real sample rate    | Printed on serial |
|-------------|-----------------------|----------------|---------------------|-------------------|
| `imu`       | LSM6DSV16X            | I2C_SENS       | 120 Hz              | 1 Hz              |
| `bar`       | LPS28DFW              | I2C_SENS       | 25 Hz               | 1 Hz              |
| `tof_inf`   | VL53L1X               | I2C_SENS       | up to ~50 Hz        | 1 Hz              |
| `tof_front` | VL53L8CX (4x4 zones)  | I2C_TOF        | 60 Hz               | 1 Hz              |
| `temp`      | MCP9808               | I2C_SENS       | 1 Hz (polling)      | 1 Hz              |
| `camera`    | camera, through the DSP | DSP_SYNC pulse | one pulse per image (25 Hz PAL) | 1 Hz |

### imu: accelerometer + gyroscope (LSM6DSV16X)

```cpp
struct imu_data_t {
  float ax, ay, az;  // acceleration, m/s^2
  float gx, gy, gz;  // rotation rate, deg/s
};

int imu_begin();
int imu_update();
int imu_get(imu_data_t *out);  // copies the last valid sample; FW_ERR_NOT_READY if none yet
```

### bar: barometer (LPS28DFW)

```cpp
int   bar_begin();
int   bar_update();
float bar_get_hpa();         // air pressure, hPa
float bar_get_altitude_m();  // altitude relative to the pressure read in bar_begin(), m
```

### tof_inf: bottom distance sensor, distance to the ground (VL53L1X)

```cpp
int      tof_inf_begin();
int      tof_inf_update();
uint16_t tof_inf_get_mm();  // distance to the ground, mm (0 to 4000)
```

### tof_front: front distance sensor, obstacles ahead (VL53L8CX, 4x4 zones)

```cpp
int      tof_front_begin();
int      tof_front_update();
uint16_t tof_front_get_mm();                   // closest obstacle over the 16 zones, mm (0 to 4000)
uint16_t tof_front_get_zone_mm(uint8_t zone);  // one zone, 0 to 15; returns 0 if zone > 15
```

### temp: temperature sensor (MCP9808)

```cpp
int   temp_begin();
int   temp_update();
float temp_get_celsius();  // temperature, degC
```

### camera: images of the camera, seen through the DSP

The camera is wired to the DSP (TI LCDK C6748), **not** to the ESP32. The DSP sends one pulse on
`DSP_SYNC` at the start of every image. The ESP32 counts these pulses: the count is the image
number (`n_img`) that will tag every measurement sent to the DSP.

```cpp
int      camera_begin();
int      camera_update();            // FW_ERR_TIMEOUT if no pulse arrived in the last 1 s
uint32_t camera_get_frames();        // DSP_SYNC pulses (images) since boot
uint32_t camera_get_last_sync_us();  // micros() of the last pulse
```

## Error codes

Defined once in `fw/lib/common/fw_errors.h`. No C++ exceptions.

| Code | Name               | Meaning                                                     |
|-----:|--------------------|-------------------------------------------------------------|
| 0    | `FW_OK`            | everything fine                                             |
| 1    | `FW_ERR_NOT_FOUND` | the device did not answer (wrong address, not connected)    |
| 2    | `FW_ERR_TIMEOUT`   | a read started, or a pulse was expected, and nothing came   |
| 3    | `FW_ERR_BAD_DATA`  | data out of range, wrong length or wrong checksum           |
| 4    | `FW_ERR_NOT_READY` | called before `begin()`, or there is no valid sample yet    |

## Pin mapping

Taken from the schematic (`Grupo_2/ESP32.kicad_sch`) and the final schematic + PCB review.
Pin numbers are written **only** in `fw/lib/common/board_pins.h`; libraries include it and never
hard-code a number. Everything is 3.3 V; no level shifters.

Buses, started in `main.cpp` before any `<name>_begin()`:

| Bus      | Arduino object | SDA  | SCL  | Speed   | Devices (7-bit address)                                     |
|----------|----------------|------|------|---------|-------------------------------------------------------------|
| I2C_SENS | `Wire`         | IO21 | IO22 | 400 kHz | VL53L1X 0x29, MCP9808 0x18, LPS28DFW 0x5C, LSM6DSV16X 0x6B   |
| I2C_TOF  | `Wire1`        | IO32 | IO33 | 400 kHz | VL53L8CX 0x29                                               |

The two ToF sensors share address 0x29 but sit on different buses, so there is no clash.

Other signals:

| Component | Signal          | ESP32 pin | Direction              |
|-----------|-----------------|-----------|------------------------|
| imu       | INT1_IMU        | IO25      | input (interrupt)      |
| imu       | INT2_IMU        | IO26      | input (spare)          |
| bar       | INT_ALT         | IO27      | input (interrupt)      |
| tof_inf   | XSHUT_TOF_INF   | IO18      | output, LOW = off      |
| tof_inf   | INT_TOF_INF     | IO35      | input only, no internal pull-up (pull-up on the SATEL board) |
| tof_front | LPN_TOF_FRONT   | IO19      | output                 |
| tof_front | PWR_EN_FRONT    | IO2       | output (boot strapping pin) |
| tof_front | INT_TOF_FRONT   | IO34      | input only, no internal pull-up (pull-up on the SATEL board) |
| tof_front | SYNC_TOF_FRONT  | IO23      | output (not used yet)  |
| camera    | DSP_SYNC        | IO4       | input, pulse per image |
| (status)  | LED_ESTADO      | IO12      | output (boot strapping pin) |

Reserved, nobody else uses them:

| Pins        | Use                                                        |
|-------------|------------------------------------------------------------|
| IO1 / IO3   | UART0: USB of the DevKit, contract serial output           |
| IO13 / IO14 | UART1 TX / RX: data link with the DSP (460800 baud)        |
| IO17 / IO16 | UART2 TX / RX: Pixhawk flight controller (MAVLink v2)      |

## Serial format

UART0 at 115200 baud, one line per message, fields separated by one space, floats with 2 decimals.

| Message | Format                          | When                                       |
|---------|---------------------------------|--------------------------------------------|
| Version | `[fw] name=g2fw version=1.0.0`  | once, in `setup()`, just before the banner |
| Banner  | `FPF-LAB READY`                 | once, the **last** line of `setup()`       |
| Update  | `[<name>] key=value key=value`  | once per second per component (`millis()`) |
| Error   | `[<name>] ERR code=<n>`         | when `begin()` or `update()` returns != 0  |

One example line per component:

```text
[fw] name=g2fw version=1.0.0
FPF-LAB READY
[imu] ax=0.02 ay=-0.01 az=9.81 gx=0.10 gy=-0.20 gz=0.00
[bar] hpa=1013.25 alt_m=0.00
[tof_inf] mm=850
[tof_front] mm=1200
[temp] c=21.50
[camera] frames=25
[temp] ERR code=2
```

## Rules

- **Non-blocking**: no `delay()` anywhere in `fw/lib` (CI fails if it finds one). Use `millis()`.
- **Interrupts**: an interrupt routine only saves `micros()` and sets a flag. The I2C read
  happens later, in `<name>_update()`. Never use `Wire` inside an interrupt.
- **Bounded buffers**: fixed-size arrays only, always check the index. No `String` in `fw/lib`.
- **Validate input**: check length and range of every value read from a sensor before using it.
- **Own folder only**: each pair changes only `fw/lib/<name>/` and its own `setupX()`/`loopX()`
  slot in `fw/src/main.cpp`.
- **Same header for mock and real driver**: the `.h` never changes when the mock is replaced.

## Not in this version

The binary frame to the DSP over UART1 (header `0xAA 0x55`, type, length, sequence, data, CRC-16)
and the MAVLink v2 messages to the Pixhawk over UART2 come later, in their own issues. They will
use the getters above, so the component APIs do not change.
