//
// Created by Matthew Oduamafu on 08/11/2025.
//
/*
 * bme280_sensor_driver.h
 * Driver for BME280/BMP280 sensor
 */

#ifndef BME280_SENSOR_DRIVER_H
#define BME280_SENSOR_DRIVER_H

#include "stm32f0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// BME280 I2C Address
#define BME280_I2C_ADDR_PRIMARY    0x76U << 1
#define BME280_I2C_ADDR_SECONDARY  0x77U << 1

// Register addresses
#define BME280_REG_ID              0xD0U
#define BME280_REG_RESET           0xE0U
#define BME280_REG_CTRL_HUM        0xF2U
#define BME280_REG_STATUS          0xF3U
#define BME280_REG_CTRL_MEAS       0xF4U
#define BME280_REG_CONFIG          0xF5U
#define BME280_REG_PRESS_MSB       0xF7U
#define BME280_REG_TEMP_MSB        0xFAU
#define BME280_REG_HUM_MSB         0xFDU

// Calibration registers
#define BME280_REG_CALIB00         0x88U
#define BME280_REG_CALIB26         0xE1U

// Chip ID
#define BME280_CHIP_ID             0x60U
#define BMP280_CHIP_ID             0x58U

// Oversampling settings
#define BME280_OVERSAMPLING_SKIP   0x00U
#define BME280_OVERSAMPLING_1X     0x01U
#define BME280_OVERSAMPLING_2X     0x02U
#define BME280_OVERSAMPLING_4X     0x03U
#define BME280_OVERSAMPLING_8X     0x04U
#define BME280_OVERSAMPLING_16X    0x05U

// Operating modes
#define BME280_MODE_SLEEP          0x00U
#define BME280_MODE_FORCED         0x01U
#define BME280_MODE_NORMAL         0x03U

// Filter settings
#define BME280_FILTER_OFF          0x00U
#define BME280_FILTER_2            0x01U
#define BME280_FILTER_4            0x02U
#define BME280_FILTER_8            0x03U
#define BME280_FILTER_16           0x04U

// Standby time
#define BME280_STANDBY_0_5MS       0x00U
#define BME280_STANDBY_62_5MS      0x01U
#define BME280_STANDBY_125MS       0x02U
#define BME280_STANDBY_250MS       0x03U
#define BME280_STANDBY_500MS       0x04U
#define BME280_STANDBY_1000MS      0x05U
#define BME280_STANDBY_10MS        0x06U
#define BME280_STANDBY_20MS        0x07U

// Calibration data structure
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
    int32_t t_fine;
} BME280_CalibData;

// Sensor configuration
typedef struct {
    uint8_t osrs_t;      // Temperature oversampling
    uint8_t osrs_p;      // Pressure oversampling
    uint8_t osrs_h;      // Humidity oversampling
    uint8_t mode;        // Operating mode
    uint8_t filter;      // Filter coefficient
    uint8_t standby;     // Standby time
} BME280_Config;

// BME280 Handle
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    uint8_t chip_id;
    bool is_bme280;      // true for BME280, false for BMP280
    BME280_CalibData calib;
    BME280_Config config;
} BME280_Handle;

// Sensor data structure
typedef struct {
    float temperature;   // °C
    float pressure;      // hPa
    float humidity;      // %RH (0 if BMP280)
    float altitude;      // meters (calculated)
} BME280_Data;

// Function prototypes
HAL_StatusTypeDef BME280_Init(BME280_Handle *hbme, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BME280_SetConfig(BME280_Handle *hbme, BME280_Config *config);
HAL_StatusTypeDef BME280_ReadSensorData(BME280_Handle *hbme, BME280_Data *data);
HAL_StatusTypeDef BME280_ReadTemperature(BME280_Handle *hbme, float *temperature);
HAL_StatusTypeDef BME280_ReadPressure(BME280_Handle *hbme, float *pressure);
HAL_StatusTypeDef BME280_ReadHumidity(BME280_Handle *hbme, float *humidity);
float BME280_CalculateAltitude(float pressure, float sea_level_pressure);
HAL_StatusTypeDef BME280_SoftReset(BME280_Handle *hbme);
bool BME280_IsMeasuring(BME280_Handle *hbme);


#endif //BME280_SENSOR_DRIVER_H