# g2fw: firmware contract

This file is the **single source of truth** for the firmware. If the code and this README
disagree, the code is wrong, or this README is changed first in its own PR.

- **Board:** ESP32-DevKitC-32E, the module on our drone PCB (`board = esp32dev`). The DOIT ESP32
  DEVKIT V1 used on the breadboard has the same chip and GPIO numbers.
- **Module:** ESP32-WROOM-32E, without PSRAM. IO16 and IO17 (INT_TOF_INF, INT_ALT) are free only
  on WROOM modules: a WROVER module uses them for its PSRAM. Do not swap the module without
  moving those two signals.
- **PlatformIO env:** `esp32` (`pio run -e esp32`). CI builds it in the job `build (esp32)`.
- **Serial port (UART0, USB of the DevKit):** 115200 baud. This is the output the tests read.
- **Firmware version:** `1.0.0`.

## Components

Every component lives in `fw/lib/<name>/` (`<name>.h` + `<name>.cpp`) and follows the same pattern:

```cpp
#include <fw_errors.h>

int <name>_begin();            // call once in setup(). May wait (see Rules). Returns FW_OK or an
                               // error code.
int <name>_update();           // call on every loop(). Never waits. Returns FW_OK if there was
                               // nothing to do or the read worked, or an error code if it failed.
int <name>_get(<type> *out);   // copies the LAST VALID sample into *out and returns FW_OK.
                               // Before the first valid sample it returns FW_ERR_NOT_READY and
                               // leaves *out untouched. It never touches the hardware.
```

**One rule for every component:** data is read only through `<name>_get()`, and "no data yet"
is always `FW_ERR_NOT_READY`, never a special value (0 mm and 0 m are real readings).

The I2C buses are started once in `main.cpp` (see *Pin mapping*); a library never calls
`Wire.begin()` or `Wire1.begin()`.

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
struct bar_data_t {
  float hpa;         // air pressure, hPa
  float altitude_m;  // altitude relative to the pressure read in bar_begin(), m
};

int bar_begin();
int bar_update();
int bar_get(bar_data_t *out);
```

### tof_inf: bottom distance sensor, distance to the ground (VL53L1X)

```cpp
int tof_inf_begin();
int tof_inf_update();
int tof_inf_get(uint16_t *mm);  // distance to the ground, mm (0 to 4000)
```

### tof_front: front distance sensor, obstacles ahead (VL53L8CX, 4x4 zones)

```cpp
struct tof_front_data_t {
  uint16_t min_mm;       // closest obstacle over the 16 zones, mm (0 to 4000); 4000 if all zones are clear
  uint16_t zone_mm[16];  // distance of each zone, mm, in the order given by the ST driver;
                         // 4000 = no target in that zone (path clear up to max range)
};

int tof_front_begin();   // may wait up to 5 s while the driver starts (see Rules)
int tof_front_update();
int tof_front_get(tof_front_data_t *out);
```

A zone is valid only with ST target status 5 or 9 (UM3109). Status 255 (no target) is reported as 4000.
The SATEL-VL53L8 board is powered at **5 V** through EXT_5V0; its signals are 3.3 V.

### temp: temperature sensor (MCP9808)

```cpp
int temp_begin();
int temp_update();
int temp_get(float *celsius);  // temperature, degC
```

### camera: images of the camera, seen through the DSP

The camera is wired to the DSP (TI LCDK C6748), **not** to the ESP32. The DSP sends one pulse on
`DSP_SYNC` at the start of every image. The ESP32 counts these pulses: the count is the image
number (`n_img`) that will tag every measurement sent to the DSP.

```cpp
struct camera_data_t {
  uint32_t frames;        // DSP_SYNC pulses (images) since boot
  uint32_t last_sync_us;  // micros() of the last pulse
};

int camera_begin();
int camera_update();  // FW_ERR_TIMEOUT if no pulse arrived in the last 1 s
int camera_get(camera_data_t *out);
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

Taken from the board we sent to fabrication (rev B, 19/11/2026): schematic and PCB give the same
pins. The pins of rev A (first routing) are **no longer valid**.
Pin numbers are written **only** in `fw/lib/common/board_pins.h`; libraries include it and never
hard-code a number. All signals are 3.3 V; no level shifters on our board. The only 5 V supply is
the SATEL-VL53L8 (EXT_5V0).

Buses, started in `main.cpp` before any `<name>_begin()`:

| Bus      | Arduino object | SDA  | SCL  | Speed   | Devices (7-bit address)                                     |
|----------|----------------|------|------|---------|-------------------------------------------------------------|
| I2C_SENS | `Wire`         | IO21 | IO22 | 400 kHz | VL53L1X 0x29, MCP9808 0x18, LPS28DFW 0x5C, LSM6DSV16X 0x6B   |
| I2C_TOF  | `Wire1`        | IO27 | IO26 | 400 kHz | VL53L8CX 0x29                                               |

The two ToF sensors share address 0x29 but sit on different buses, so there is no clash.

Other signals:

| Component | Signal          | ESP32 pin | Direction              |
|-----------|-----------------|-----------|------------------------|
| imu       | INT1_IMU        | IO19      | input (interrupt)      |
| imu       | INT2_IMU        | IO18      | input (spare)          |
| bar       | INT_ALT         | IO17      | input (interrupt)      |
| tof_inf   | XSHUT_TOF_INF   | IO4       | output, LOW = off      |
| tof_inf   | INT_TOF_INF     | IO16      | input (pull-up on the SATEL board) |
| tof_front | LPN_TOF_FRONT   | IO13      | output                 |
| tof_front | PWR_EN_FRONT    | IO12      | output, LOW at boot (see below) |
| tof_front | INT_TOF_FRONT   | IO34      | input only, no internal pull-up (pull-up on the SATEL board) |
| tof_front | SYNC_TOF_FRONT  | IO14      | keep as INPUT until it is used (see below) |
| camera    | DSP_SYNC        | IO33      | input, pulse per image |
| (status)  | LED_ESTADO      | IO2       | output                 |

Boot (strapping) pins, handle with care:

- **IO12 (PWR_EN_FRONT)** is MTDI: if it is HIGH at boot, the ESP32 sets its flash to 1.8 V and
  does not start. The 100 kΩ pull-down of the SATEL board keeps it LOW. Never add a pull-up.
- **IO2 (LED_ESTADO)** must be LOW or floating to enter download mode; the LED to GND keeps it
  LOW. On the breadboard DOIT board, its blue LED is now the status LED.
- **IO14 (SYNC_TOF_FRONT)** outputs pulses during boot. Harmless, because the sensor is still off
  (PWR_EN LOW), but configure it as INPUT until the sync feature is used.

Reserved, nobody else uses them:

| Pins        | Use                                                                   |
|-------------|-----------------------------------------------------------------------|
| IO1 / IO3   | UART0: USB of the DevKit, contract serial output                      |
| IO32 / IO35 | UART1 TX (DSP_LINK_A) / RX (DSP_LINK_B, input only): DSP, 460800 baud |
| IO25 / IO39 | UART2 TX / RX (input only): Pixhawk flight controller (MAVLink v2)    |

## Serial format

UART0 at 115200 baud, one line per message, fields separated by one space, floats with 2 decimals.

| Message | Format                          | When                                       |
|---------|---------------------------------|--------------------------------------------|
| Version | `[fw] name=g2fw version=1.0.0`  | once, in `setup()`, just before the banner |
| Banner  | `FPF-LAB READY`                 | once, the **last** line of `setup()`       |
| Update  | `[<name>] key=value key=value`  | once per second per component (`millis()`), only when `<name>_get()` returns `FW_OK` |
| Error   | `[<name>] ERR code=<n>`         | when `begin()` or `update()` returns != 0, at most once per second per component |

While `<name>_get()` returns `FW_ERR_NOT_READY`, nothing is printed for that component.

**Camera without the DSP.** With no DSP connected (in class and in Wokwi) no `DSP_SYNC` pulse
arrives, so `camera_update()` returns `FW_ERR_TIMEOUT` and `[camera] ERR code=2` is printed once
per second. This is the **expected** output, not a bug, and the VnV scenarios must accept it.

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
[camera] ERR code=2
```

## Rules

- **Non-blocking**: `<name>_update()` and the getters never wait. Use `millis()` instead of
  `delay()`.
- **One-time waits only in `<name>_begin()`**: it runs once, in `setup()`, so a wait there only
  delays boot (sensor boot time, a firmware upload...). Every `delay()` inside a
  `<name>_begin()` is marked on the same line with `// setup-only`, e.g.
  `delay(10);  // setup-only`. CI's `lint` job rejects any `delay(` in `fw/lib` without that mark.
- **`tof_front` start-up**: `tof_front_begin()` waits while the ST driver uploads its ~84 KB
  firmware to the VL53L8CX over I2C (`WaitMs()`). The ST driver comes in as an external
  library, not inside `fw/lib`. Before calling the ST init, `tof_front_begin()` checks for an
  ACK at 0x29 and returns `FW_ERR_NOT_FOUND` at once if there is none (the ST driver would retry
  forever). The whole start-up is bounded to 5 s, or it returns `FW_ERR_TIMEOUT`. Expected time
  ≈2 s at 400 kHz (measured in test 05); VnV waits up to 5 s for the banner.
- **Interrupts**: an interrupt routine is marked `IRAM_ATTR` and only saves `micros()` and sets
  a flag. Every variable it writes is `volatile`. The main code copies them with interrupts
  disabled (`portENTER_CRITICAL()` / `portEXIT_CRITICAL()`) when they are wider than 32 bits or
  when several of them must be read together. The I2C read happens later, in `<name>_update()`.
  Never use `Wire` inside an interrupt.
- **Bounded buffers**: fixed-size arrays only, always check the index. No `String` in `fw/lib`.
- **Validate input**: check length and range of every value read from a sensor before using it.
- **Own folder only**: each pair changes only `fw/lib/<name>/` and its own `setupX()`/`loopX()`
  slot in `fw/src/main.cpp`.
- **Same header for mock and real driver**: the `.h` never changes when the mock is replaced.

## Not in this version

The binary frame to the DSP over UART1 (header `0xAA 0x55`, type, length, sequence, data, CRC-16)
and the MAVLink v2 messages to the Pixhawk over UART2 come later, in their own issues. They will
use the getters above, so the component APIs do not change.
