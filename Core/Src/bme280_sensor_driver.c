//
// Created by Matthew Oduamafu on 08/11/2025.
//
/*
 * bme280_sensor_driver.c
 * Implementation of BME280/BMP280 sensor driver
 */

#include "bme280_sensor_driver.h"
#include <math.h>

// Private function prototypes
static HAL_StatusTypeDef BME280_ReadCalibrationData(BME280_Handle *hbme);
static int32_t BME280_CompensateTemperature(BME280_Handle *hbme, int32_t adc_T);
static uint32_t BME280_CompensatePressure(BME280_Handle *hbme, int32_t adc_P);
static uint32_t BME280_CompensateHumidity(BME280_Handle *hbme, int32_t adc_H);

// Initialize BME280 sensor
HAL_StatusTypeDef BME280_Init(BME280_Handle *hbme, I2C_HandleTypeDef *hi2c) {
    hbme->hi2c = hi2c;
    hbme->address = BME280_I2C_ADDR_PRIMARY;

    // Check primary address
    if (HAL_I2C_IsDeviceReady(hbme->hi2c, hbme->address, 3, 100) != HAL_OK) {
        // Try secondary address
        hbme->address = BME280_I2C_ADDR_SECONDARY;
        if (HAL_I2C_IsDeviceReady(hbme->hi2c, hbme->address, 3, 100) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    // Read chip ID
    HAL_I2C_Mem_Read(hbme->hi2c, hbme->address, BME280_REG_ID, 1, &hbme->chip_id, 1, 100);

    // Determine sensor type
    if (hbme->chip_id == BME280_CHIP_ID) {
        hbme->is_bme280 = true;
    } else if (hbme->chip_id == BMP280_CHIP_ID) {
        hbme->is_bme280 = false;
    } else {
        return HAL_ERROR;
    }

    // Soft reset
    BME280_SoftReset(hbme);
    HAL_Delay(10);

    // Read calibration data
    if (BME280_ReadCalibrationData(hbme) != HAL_OK) {
        return HAL_ERROR;
    }

    // Default configuration
    BME280_Config config = {
        .osrs_t = BME280_OVERSAMPLING_1X,
        .osrs_p = BME280_OVERSAMPLING_1X,
        .osrs_h = BME280_OVERSAMPLING_1X,
        .mode = BME280_MODE_NORMAL,
        .filter = BME280_FILTER_OFF,
        .standby = BME280_STANDBY_500MS
    };

    return BME280_SetConfig(hbme, &config);
}

// Set sensor configuration
HAL_StatusTypeDef BME280_SetConfig(BME280_Handle *hbme, BME280_Config *config) {
    hbme->config = *config;

    // Set humidity oversampling (BME280 only)
    if (hbme->is_bme280) {
        uint8_t ctrl_hum = config->osrs_h & 0x07U;
        HAL_I2C_Mem_Write(hbme->hi2c, hbme->address, BME280_REG_CTRL_HUM, 1, &ctrl_hum, 1, 100);
    }

    // Set config register
    uint8_t cfg = ((config->standby << 5U) | (config->filter << 2U)) & 0xFCU;
    HAL_I2C_Mem_Write(hbme->hi2c, hbme->address, BME280_REG_CONFIG, 1, &cfg, 1, 100);

    // Set control register
    uint8_t ctrl_meas = ((config->osrs_t << 5U) | (config->osrs_p << 2U) | config->mode) & 0xFFU;
    HAL_I2C_Mem_Write(hbme->hi2c, hbme->address, BME280_REG_CTRL_MEAS, 1, &ctrl_meas, 1, 100);

    return HAL_OK;
}

// Read all sensor data
HAL_StatusTypeDef BME280_ReadSensorData(BME280_Handle *hbme, BME280_Data *data) {
    uint8_t raw_data[8];

    // Read all data registers
    if (HAL_I2C_Mem_Read(hbme->hi2c, hbme->address, BME280_REG_PRESS_MSB, 1, raw_data, 8, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Parse raw data
    int32_t adc_P = ((uint32_t)raw_data[0U] << 12U) | ((uint32_t)raw_data[1U] << 4U) | ((uint32_t)raw_data[2U] >> 4U);
    int32_t adc_T = ((uint32_t)raw_data[3U] << 12U) | ((uint32_t)raw_data[4U] << 4U) | ((uint32_t)raw_data[5U] >> 4U);
    int32_t adc_H = ((uint32_t)raw_data[6U] << 8U) | (uint32_t)raw_data[7U];

    // Compensate temperature (must be done first)
    int32_t temp = BME280_CompensateTemperature(hbme, adc_T);
    data->temperature = temp / 100.0f;

    // Compensate pressure
    uint32_t press = BME280_CompensatePressure(hbme, adc_P);
    data->pressure = press / 25600.0f;

    // Compensate humidity (BME280 only)
    if (hbme->is_bme280) {
        uint32_t hum = BME280_CompensateHumidity(hbme, adc_H);
        data->humidity = hum / 1024.0f;
    } else {
        data->humidity = 0.0f;
    }

    // Calculate altitude
    data->altitude = BME280_CalculateAltitude(data->pressure, 1013.25f);

    return HAL_OK;
}

// Read temperature only
HAL_StatusTypeDef BME280_ReadTemperature(BME280_Handle *hbme, float *temperature) {
    BME280_Data data;
    HAL_StatusTypeDef status = BME280_ReadSensorData(hbme, &data);
    *temperature = data.temperature;
    return status;
}

// Read pressure only
HAL_StatusTypeDef BME280_ReadPressure(BME280_Handle *hbme, float *pressure) {
    BME280_Data data;
    HAL_StatusTypeDef status = BME280_ReadSensorData(hbme, &data);
    *pressure = data.pressure;
    return status;
}

// Read humidity only
HAL_StatusTypeDef BME280_ReadHumidity(BME280_Handle *hbme, float *humidity) {
    BME280_Data data;
    HAL_StatusTypeDef status = BME280_ReadSensorData(hbme, &data);
    *humidity = data.humidity;
    return status;
}

// Calculate altitude from pressure
float BME280_CalculateAltitude(float pressure, float sea_level_pressure) {
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

// Soft reset
HAL_StatusTypeDef BME280_SoftReset(BME280_Handle *hbme) {
    uint8_t reset_cmd = 0xB6U;
    return HAL_I2C_Mem_Write(hbme->hi2c, hbme->address, BME280_REG_RESET, 1, &reset_cmd, 1, 100);
}

// Check if sensor is measuring
bool BME280_IsMeasuring(BME280_Handle *hbme) {
    uint8_t status;
    HAL_I2C_Mem_Read(hbme->hi2c, hbme->address, BME280_REG_STATUS, 1, &status, 1, 100);
    return (status & 0x08U) != 0;
}

// Read calibration data
static HAL_StatusTypeDef BME280_ReadCalibrationData(BME280_Handle *hbme) {
    uint8_t calib_data[26U];

    // Read temperature and pressure calibration
    if (HAL_I2C_Mem_Read(hbme->hi2c, hbme->address, BME280_REG_CALIB00, 1, calib_data, 26, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    hbme->calib.dig_T1 = (calib_data[1U] << 8U) | calib_data[0U];
    hbme->calib.dig_T2 = (calib_data[3U] << 8U) | calib_data[2U];
    hbme->calib.dig_T3 = (calib_data[5U] << 8U) | calib_data[4U];
    hbme->calib.dig_P1 = (calib_data[7U] << 8U) | calib_data[6U];
    hbme->calib.dig_P2 = (calib_data[9U] << 8U) | calib_data[8U];
    hbme->calib.dig_P3 = (calib_data[11U] << 8U) | calib_data[10U];
    hbme->calib.dig_P4 = (calib_data[13U] << 8U) | calib_data[12U];
    hbme->calib.dig_P5 = (calib_data[15U] << 8U) | calib_data[14U];
    hbme->calib.dig_P6 = (calib_data[17U] << 8U) | calib_data[16U];
    hbme->calib.dig_P7 = (calib_data[19U] << 8U) | calib_data[18U];
    hbme->calib.dig_P8 = (calib_data[21U] << 8U) | calib_data[20U];
    hbme->calib.dig_P9 = (calib_data[23U] << 8U) | calib_data[22U];
    hbme->calib.dig_H1 = calib_data[25U];

    // Read humidity calibration (BME280 only)
    if (hbme->is_bme280) {
        uint8_t hum_calib[7U];
        HAL_I2C_Mem_Read(hbme->hi2c, hbme->address, BME280_REG_CALIB26, 1, hum_calib, 7, 100);

        hbme->calib.dig_H2 = (hum_calib[1U] << 8U) | hum_calib[0U];
        hbme->calib.dig_H3 = hum_calib[2U];
        hbme->calib.dig_H4 = (hum_calib[3U] << 4U) | (hum_calib[4U] & 0x0FU);
        hbme->calib.dig_H5 = (hum_calib[5U] << 4U) | ((hum_calib[4U] >> 4U) & 0x0FU);
        hbme->calib.dig_H6 = hum_calib[6U];
    }

    return HAL_OK;
}

// Compensate temperature
static int32_t BME280_CompensateTemperature(BME280_Handle *hbme, int32_t adc_T) {
    int32_t var1, var2;

    var1 = ((((adc_T >> 3U) - ((int32_t)hbme->calib.dig_T1 << 1U))) * ((int32_t)hbme->calib.dig_T2)) >> 11U;
    var2 = (((((adc_T >> 4U) - ((int32_t)hbme->calib.dig_T1)) * ((adc_T >> 4U) - ((int32_t)hbme->calib.dig_T1))) >> 12U) *
            ((int32_t)hbme->calib.dig_T3)) >> 14U;

    hbme->calib.t_fine = var1 + var2;

    return (hbme->calib.t_fine * 5U + 128U) >> 8U;
}

// Compensate pressure
static uint32_t BME280_CompensatePressure(BME280_Handle *hbme, int32_t adc_P) {
    int64_t var1, var2, p;

    var1 = ((int64_t)hbme->calib.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)hbme->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)hbme->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)hbme->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)hbme->calib.dig_P3) >> 8U) + ((var1 * (int64_t)hbme->calib.dig_P2) << 12U);
    var1 = (((((int64_t)1U) << 47U) + var1)) * ((int64_t)hbme->calib.dig_P1) >> 33U;

    if (var1 == 0) return 0;

    p = 1048576U - adc_P;
    p = (((p << 31U) - var2) * 3125U) / var1;
    var1 = (((int64_t)hbme->calib.dig_P9) * (p >> 13U) * (p >> 13U)) >> 25U;
    var2 = (((int64_t)hbme->calib.dig_P8) * p) >> 19U;

    p = ((p + var1 + var2) >> 8U) + (((int64_t)hbme->calib.dig_P7) << 4U);

    return (uint32_t)p;
}

// Compensate humidity
static uint32_t BME280_CompensateHumidity(BME280_Handle *hbme, int32_t adc_H) {
    int32_t v_x1_u32r;

    v_x1_u32r = (hbme->calib.t_fine - ((int32_t)76800U));
    v_x1_u32r = (((((adc_H << 14U) - (((int32_t)hbme->calib.dig_H4) << 20U) -
                    (((int32_t)hbme->calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384U)) >> 15U) *
                 (((((((v_x1_u32r * ((int32_t)hbme->calib.dig_H6)) >> 10U) *
                      (((v_x1_u32r * ((int32_t)hbme->calib.dig_H3)) >> 11U) + ((int32_t)32768U))) >> 10U) +
                    ((int32_t)2097152U)) * ((int32_t)hbme->calib.dig_H2) + 8192U) >> 14U));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15U) * (v_x1_u32r >> 15U)) >> 7U) *
                                ((int32_t)hbme->calib.dig_H1)) >> 4U));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400U ? 419430400U : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12U);
}