# STM32 BME280/BMP280 Sensor Driver

Reusable, STM32/HAL-friendly I2C driver for Bosch BME280/BMP280 sensors.

> **Project focus:** this repository exists mainly to provide the `bme280_sensor_driver.[ch]` files as a small, self-contained driver that you can drop into **any** STM32 (or similar HAL-based) project.

The rest of the code (startup, clocks, LCD, etc.) is just a reference application around the driver.

---

## 1. Project Overview

This project provides a lightweight C driver for the Bosch BME280/BMP280 environmental sensors:

- Written for STM32 using the STM32 HAL I2C API
- Designed to be portable to other MCUs and I2C abstractions
- Minimal dependencies beyond `<stdint.h>`, `<stdbool.h>`, and HAL I2C
- Clear separation between:
  - Low-level I2C register access
  - High-level configuration and data conversion

The reusable artifact is the `Core/Inc/bme280_sensor_driver.h` and `Core/Src/bme280_sensor_driver.c` pair.

---

## 2. Driver Features

Key capabilities of `bme280_sensor_driver.[ch]`:

- **BME280 / BMP280 support**
  - Supports both humidity-capable BME280 and pressure/temperature-only BMP280
  - Internally tracks if the device is BME280 vs BMP280

- **Flexible oversampling / filter configuration**
  - Temperature, pressure, and humidity oversampling settings
  - IIR filter coefficient configuration
  - Standby time configuration for normal mode

- **Handle-based configuration (`BME280_Handle`)**
  - Stores:
    - HAL I2C handle reference
    - Sensor I2C address
    - Detected chip ID and type
    - Calibration data loaded from the sensor
    - Runtime configuration (mode, oversampling, filter, standby)

- **Typed sensor data struct (`BME280_Data`)**
  - Temperature (°C)
  - Pressure (hPa)
  - Humidity (%RH, 0 if BMP280)
  - Altitude (m, calculated)

- **Altitude calculation helper**
  - Convenience helper using the standard barometric formula
  - Takes sea-level reference pressure as parameter

- **STM32 HAL I2C dependency only**
  - Uses STM32 HAL types and functions (`I2C_HandleTypeDef`, `HAL_StatusTypeDef`, etc.)
  - Easy to adapt to another I2C abstraction by replacing a small number of calls

---

## 3. Quick Start / Integration

The goal is to make integration a simple copy-and-wire task.

### 3.1. Files to copy

At minimum, copy these into your project:

- `Core/Inc/bme280_sensor_driver.h` → your `Inc/` or `include/` folder
- `Core/Src/bme280_sensor_driver.c` → your `Src/` or `src/` folder

Make sure your compiler include paths let you do:

```c
#include "bme280_sensor_driver.h"
```

### 3.2. HAL & I2C requirements

Your project must already provide:

- STM32 HAL headers and I2C support (e.g. `stm32f0xx_hal.h`, `stm32f0xx_hal_i2c.h`)
- An initialized `I2C_HandleTypeDef` (e.g. `hi2c1`)
- Correct BME280/BMP280 I2C pins, alternate functions, and clocks (configured via CubeMX or manually)

### 3.3. Initialize `BME280_Handle`

Typical pattern in your application code (e.g. `main.c` or a sensor module):

1. Declare a global or static handle:
   ```c
   BME280_Handle hbme;
   ```
2. Call the init function once after `hi2cX` is initialized:
   ```c
   extern I2C_HandleTypeDef hi2c1;

   if (BME280_Init(&hbme, &hi2c1) != HAL_OK) {
       // handle error (wrong chip ID, I2C problem, etc.)
   }
   ```

The init routine:

- Stores the I2C handle and sensor address in `hbme`
- Reads and checks the chip ID (`BME280_CHIP_ID` / `BMP280_CHIP_ID`)
- Loads the sensor calibration data into `hbme.calib`

### 3.4. Configure oversampling and mode

Use the `BME280_Config` structure and `BME280_SetConfig` to configure the sensor:

```c
BME280_Config cfg = {
    .osrs_t = BME280_OVERSAMPLING_1X,
    .osrs_p = BME280_OVERSAMPLING_1X,
    .osrs_h = BME280_OVERSAMPLING_1X,
    .mode   = BME280_MODE_NORMAL,
    .filter = BME280_FILTER_OFF,
    .standby = BME280_STANDBY_1000MS,
};

BME280_SetConfig(&hbme, &cfg);
```

Adjust oversampling, filter and standby based on your noise vs. power trade-off.

### 3.5. Basic read loop

In your main loop or RTOS task:

```c
BME280_Data data;

for (;;) {
    if (BME280_ReadSensorData(&hbme, &data) == HAL_OK) {
        float altitude = BME280_CalculateAltitude(data.pressure, 1013.25f); // example sea-level pressure
        data.altitude = altitude;

        // use data.temperature, data.pressure, data.humidity, data.altitude
    }

    HAL_Delay(1000); // or your scheduler delay
}
```

If you use **forced mode**, you may need to:

1. Trigger a new measurement via configuration
2. Wait for conversion (poll `BME280_IsMeasuring` or delay)
3. Then call `BME280_ReadSensorData`

---

## 4. Public API (from `bme280_sensor_driver.h`)

High-level functions exposed by the driver:

```c
HAL_StatusTypeDef BME280_Init(BME280_Handle *hbme, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BME280_SetConfig(BME280_Handle *hbme, BME280_Config *config);
HAL_StatusTypeDef BME280_ReadSensorData(BME280_Handle *hbme, BME280_Data *data);
HAL_StatusTypeDef BME280_ReadTemperature(BME280_Handle *hbme, float *temperature);
HAL_StatusTypeDef BME280_ReadPressure(BME280_Handle *hbme, float *pressure);
HAL_StatusTypeDef BME280_ReadHumidity(BME280_Handle *hbme, float *humidity);
float             BME280_CalculateAltitude(float pressure, float sea_level_pressure);
HAL_StatusTypeDef BME280_SoftReset(BME280_Handle *hbme);
bool              BME280_IsMeasuring(BME280_Handle *hbme);
```

- **Initialization:** `BME280_Init`
- **Configuration:** `BME280_SetConfig`
- **All-in-one read:** `BME280_ReadSensorData`
- **Single-quantity reads:** `BME280_ReadTemperature`, `BME280_ReadPressure`, `BME280_ReadHumidity`
- **Utility:** `BME280_CalculateAltitude`
- **Maintenance / status:** `BME280_SoftReset`, `BME280_IsMeasuring`

For register definitions, configuration enums, and structs (`BME280_CalibData`, `BME280_Config`, `BME280_Handle`, `BME280_Data`), see the header file.

---

## 5. Porting Notes

### 5.1. Different STM32 part or HAL version

- Ensure you have a valid `I2C_HandleTypeDef` (e.g. `hi2c1`, `hi2c2`)
- Configure the I2C pins, clock, and speed appropriate for your board
- As long as the STM32 HAL I2C API is available, you should not need to modify the driver code

### 5.2. Different MCU or I2C abstraction

If you are not using STM32 HAL:

1. Identify the small number of I2C operations used in `bme280_sensor_driver.c` (register read/write).
2. Replace those calls with your platform's I2C API, or wrap your API to look like HAL.
3. Replace any HAL-specific delay calls (`HAL_Delay`) with your platform delay/sleep.
4. Keep the header API the same so your application code does not change.

### 5.3. Error handling

All main functions return `HAL_StatusTypeDef` (e.g. `HAL_OK`, `HAL_ERROR`). You can:

- Map these to your own error system
- Add logging around init and read calls
- Implement retries or backoff for transient I2C errors

---

## 6. License and Future Work

### License


### Possible Improvements

- Optional SPI support for BME280/BMP280
- More example code:
  - FreeRTOS task using the driver
  - Low-power periodic sampling example
- Configurable compile-time options:
  - Fixed-point vs floating-point calculations
  - Reduced API footprint for size-constrained builds
- Unit tests or hardware-in-the-loop tests

