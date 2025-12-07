#pragma once

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "ssd1306.h"
#include "assets/face.h"
#include "mpu6050.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "shared.h"

// --- BMP280 Definitions ---
#define BMP280_I2C_ADDR      0x76       /*!< BMP280 I2C slave address (could also be 0x77) */
#define BMP280_CHIP_ID       0x58       /*!< Expected value in ID register */
#define BMP280_ID_REG        0xD0       /*!< Chip ID register */
#define BMP280_RESET_REG     0xE0       /*!< Soft reset register */
#define BMP280_CTRL_MEAS_REG 0xF4       /*!< Control and Measurement register */
#define BMP280_CONFIG_REG    0xF5       /*!< Configuration register */
#define BMP280_PRESS_MSB_REG 0xF7       /*!< Start of Pressure/Temperature data */
#define BMP280_CALIB_START   0x88       /*!< Start of Calibration Coefficient registers */
#define BMP280_CALIB_LEN     24         /*!< Length of calibration data block */

// Measurement settings (High Resolution / Normal Mode)
#define BMP280_CTRL_MEAS_VAL 0b01010111 /*! OSR_T=x2, OSR_P=x16, MODE=Normal (0b01010111) */
#define BMP280_CONFIG_VAL    0b00010000 /*! t_standby=500ms, filter=x16 */

// --- Calibration Data Structure (Required for Compensation) ---
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
    
    int32_t t_fine; // Required for pressure compensation
} bmp280_calib_param_t;

// Global storage for calibration parameters
static bmp280_calib_param_t calib_params;

static esp_err_t bmp_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_I2C_ADDR, write_buf, 2, 50 / portTICK_PERIOD_MS);
}

/**
 * @brief Reads a block of bytes from a register on the BMP280.
 */
static esp_err_t bmp_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_I2C_ADDR, &reg_addr, 1, data, len, 100 / portTICK_PERIOD_MS);
}


// =========================================================================
// BMP280 COMPENSATION ALGORITHMS (from Bosch Datasheet)
// =========================================================================

/**
 * @brief Reads all 24 bytes of factory calibration parameters from the BMP280.
 */
static esp_err_t bmp280_read_calib_params(void) {
    uint8_t calib_data[BMP280_CALIB_LEN];
    if (bmp_read_bytes(BMP280_CALIB_START, calib_data, BMP280_CALIB_LEN) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data.");
        return ESP_FAIL;
    }

    // Populate the global struct from the byte buffer (little-endian: LSB | MSB)
    calib_params.dig_T1 = (uint16_t)calib_data[1] << 8 | calib_data[0];
    calib_params.dig_T2 = (int16_t)calib_data[3] << 8 | calib_data[2];
    calib_params.dig_T3 = (int16_t)calib_data[5] << 8 | calib_data[4];
    
    calib_params.dig_P1 = (uint16_t)calib_data[7] << 8 | calib_data[6];
    calib_params.dig_P2 = (int16_t)calib_data[9] << 8 | calib_data[8];
    calib_params.dig_P3 = (int16_t)calib_data[11] << 8 | calib_data[10];
    calib_params.dig_P4 = (int16_t)calib_data[13] << 8 | calib_data[12];
    calib_params.dig_P5 = (int16_t)calib_data[15] << 8 | calib_data[14];
    calib_params.dig_P6 = (int16_t)calib_data[17] << 8 | calib_data[16];
    calib_params.dig_P7 = (int16_t)calib_data[19] << 8 | calib_data[18];
    calib_params.dig_P8 = (int16_t)calib_data[21] << 8 | calib_data[20];
    calib_params.dig_P9 = (int16_t)calib_data[23] << 8 | calib_data[22];

    return ESP_OK;
}

/**
 * @brief Compensates and returns the temperature in DegC.
 * Integer precision implementation from datasheet.
 * @param adc_T Raw 20-bit temperature value.
 * @return Temperature in 0.01 DegC (e.g., 2512 = 25.12 DegC).
 */
static int32_t bmp280_compensate_T_int32(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)calib_params.dig_T1 << 1))) * ((int32_t)calib_params.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib_params.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_params.dig_T1))) >> 12) * ((int32_t)calib_params.dig_T3)) >> 14;
    calib_params.t_fine = var1 + var2;
    T = (calib_params.t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * @brief Compensates and returns the pressure in Pa.
 * Integer precision implementation from datasheet. Requires t_fine from temperature compensation.
 * @param adc_P Raw 20-bit pressure value.
 * @return Pressure in Pa (Pascals).
 */
static uint32_t bmp280_compensate_P_int32(int32_t adc_P) {
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)calib_params.t_fine) >> 1) - 64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_params.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_params.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_params.dig_P4) << 16);
    var1 = (((calib_params.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_params.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib_params.dig_P1)) >> 15);
    if (var1 == 0) {
        return 0; // Avoid division by zero
    }
    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (p < 0x80000000) {
        p = (p << 1) / ((uint32_t)var1);
    } else {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)calib_params.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_params.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_params.dig_P7) >> 4));
    return p;
}


// =========================================================================
// BMP280 INITIALIZATION & READING
// =========================================================================

/**
 * @brief Initializes and configures the BMP280.
 */
static esp_err_t bmp280_init(void) {
    uint8_t device_id = 0;
    
    // 1. Check WHO_AM_I register
    if (bmp_read_bytes(BMP280_ID_REG, &device_id, 1) != ESP_OK || device_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid BMP280 ID: 0x%X (Expected 0x58)", device_id);
        return ESP_FAIL;
    }
    
    // 2. Perform soft reset (write 0xB6 to reset register)
    if (bmp_write_byte(BMP280_RESET_REG, 0xB6) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset to complete
    
    // 3. Read calibration parameters (critical step)
    if (bmp280_read_calib_params() != ESP_OK) return ESP_FAIL;

    // 4. Configure sensor (Filter, Standby time)
    if (bmp_write_byte(BMP280_CONFIG_REG, BMP280_CONFIG_VAL) != ESP_OK) return ESP_FAIL;
    
    // 5. Set measurement control (Oversampling, Normal Mode)
    if (bmp_write_byte(BMP280_CTRL_MEAS_REG, BMP280_CTRL_MEAS_VAL) != ESP_OK) return ESP_FAIL;
    
    ESP_LOGI(TAG, "BMP280 initialized and configured for Normal Mode.");
    return ESP_OK;
}

/**
 * @brief Reads raw Pressure and Temperature data from the sensor.
 * @param raw_P Raw compensated pressure value.
 * @param raw_T Raw compensated temperature value.
 * @return ESP_OK on success.
 */
static esp_err_t bmp280_read_raw_data(int32_t *raw_P, int32_t *raw_T) {
    uint8_t data_buf[6]; // P_MSB, P_LSB, P_XLSB, T_MSB, T_LSB, T_XLSB (6 bytes)
    
    // Read 6 bytes starting from BMP280_PRESS_MSB_REG (0xF7)
    if (bmp_read_bytes(BMP280_PRESS_MSB_REG, data_buf, 6) != ESP_OK) {
        return ESP_FAIL;
    }

    // BMP280 data is 20-bit, formatted as MSB:LSB:XLSB (with XLSB being the lowest 4 bits)
    *raw_P = (int32_t)data_buf[0] << 12 | (int32_t)data_buf[1] << 4 | (int32_t)data_buf[2] >> 4;
    *raw_T = (int32_t)data_buf[3] << 12 | (int32_t)data_buf[4] << 4 | (int32_t)data_buf[5] >> 4;

    return ESP_OK;
}
