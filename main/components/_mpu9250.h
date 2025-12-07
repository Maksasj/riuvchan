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

// --- MPU-9250 Definitions (Accel/Gyro Only) ---
#define MPU9250_I2C_ADDR     0x68       /*!< MPU-9250 main chip address */
#define MPU9250_WHO_AM_I     0x75       /*!< Device ID register, should return 0x71 */
#define MPU9250_PWR_MGMT_1   0x6B       /*!< Power Management 1 register */
#define MPU9250_INT_PIN_CFG  0x37       /*!< Used to disable all auxiliary features */
#define MPU9250_ACCEL_XOUT_H 0x3B       /*!< Start of Accel/Temp/Gyro data (14 bytes) */
#define MPU9250_DATA_LEN     14         /*!< Total bytes to read from MPU (Accel, Temp, Gyro) */
#define MPU9250_ACCEL_CONFIG 0x1C       /*!< Accel Config Register */
#define MPU9250_GYRO_CONFIG  0x1B       /*!< Gyro Config Register */

// --- Scaling Factors (Default +/- 2g, +/- 250 deg/s) ---
#define ACCEL_SENSITIVITY    16384.0f   /*!< LSB per g (for +/- 2g range) */
#define GYRO_SENSITIVITY     131.0f     /*!< LSB per deg/s (for +/- 250 deg/s range) */

/**
 * @brief Writes a single byte to a register on the MPU-9250.
 */
static esp_err_t mpu_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_I2C_ADDR, write_buf, 2, 50 / portTICK_PERIOD_MS);
}

/**
 * @brief Reads a single byte from a register on the MPU-9250.
 */
static esp_err_t mpu_read_byte(uint8_t reg_addr, uint8_t *data) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_I2C_ADDR, &reg_addr, 1, data, 1, 50 / portTICK_PERIOD_MS);
}

/**
 * @brief Reads 14 bytes of raw data (Accel, Temp, Gyro) from the MPU-9250.
 */
static esp_err_t mpu_read_data(uint8_t *data_buf) {
    // Start reading from ACCEL_XOUT_H (0x3B)
    uint8_t reg_addr = MPU9250_ACCEL_XOUT_H;
    // Use a moderate timeout for the 14-byte read
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_I2C_ADDR, &reg_addr, 1, data_buf, MPU9250_DATA_LEN, 100 / portTICK_PERIOD_MS);
}


// =========================================================================
// MPU9250 INITIALIZATION & PROCESSING (6-Axis)
// =========================================================================

/**
 * @brief Initializes and verifies the MPU-9250 (Accel/Gyro).
 */
static esp_err_t mpu9250_init(void) {
    uint8_t device_id = 0;
    
    // 1. Check WHO_AM_I register for device ID (0x71 for MPU-9250)
    if (mpu_read_byte(MPU9250_WHO_AM_I, &device_id) != ESP_OK || device_id != 0x71) {
        ESP_LOGE(TAG, "Invalid MPU-9250 ID: 0x%X (Expected 0x71)", device_id);
        return ESP_FAIL;
    }
    
    // 2. Wake up MPU-9250 (set SLEEP bit to 0 in PWR_MGMT_1)
    if (mpu_write_byte(MPU9250_PWR_MGMT_1, 0x00) != ESP_OK) return ESP_FAIL;
    
    // 3. Configure Gyro (0x1B) and Accel (0x1C) ranges (set to +/- 250 deg/s and +/- 2g)
    if (mpu_write_byte(MPU9250_GYRO_CONFIG, 0x00) != ESP_OK) return ESP_FAIL; // +/- 250 deg/s
    if (mpu_write_byte(MPU9250_ACCEL_CONFIG, 0x00) != ESP_OK) return ESP_FAIL; // +/- 2g

    // 4. Disable I2C Master Bypass mode on INT_PIN_CFG (0x37)
    // Writing 0x00 disables I2C Master mode and the Bypass feature, ensuring no communication 
    // attempts are made with the internal AK8963 magnetometer.
    if (mpu_write_byte(MPU9250_INT_PIN_CFG, 0x00) != ESP_OK) return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "MPU-9250 (Accel/Gyro 6-axis) is ready.");
    return ESP_OK;
}

/**
 * @brief Converts raw 16-bit MPU-9250 6-axis data into physical units.
 * @param mpu_raw_data 14 bytes array read from MPU-9250.
 * @param accel Array to store 3 float accelerometer values (g).
 * @param gyro Array to store 3 float gyroscope values (deg/s).
 */
static void mpu9250_process_data(const uint8_t *mpu_raw_data, float *accel, float *gyro) {
    // --- ACCELEROMETER DATA (MPU: Bytes 0-5) ---
    // The raw data is Big-Endian (MSB first)
    int16_t ax_raw = (int16_t)(mpu_raw_data[0] << 8 | mpu_raw_data[1]);
    int16_t ay_raw = (int16_t)(mpu_raw_data[2] << 8 | mpu_raw_data[3]);
    int16_t az_raw = (int16_t)(mpu_raw_data[4] << 8 | mpu_raw_data[5]);
    
    accel[0] = (float)ax_raw / ACCEL_SENSITIVITY;
    accel[1] = (float)ay_raw / ACCEL_SENSITIVITY;
    accel[2] = (float)az_raw / ACCEL_SENSITIVITY;

    // --- GYROSCOPE DATA (MPU: Bytes 8-13) ---
    // Note: Bytes 6 and 7 are Temperature data, skipped here for simplicity
    int16_t gx_raw = (int16_t)(mpu_raw_data[8] << 8 | mpu_raw_data[9]);
    int16_t gy_raw = (int16_t)(mpu_raw_data[10] << 8 | mpu_raw_data[11]);
    int16_t gz_raw = (int16_t)(mpu_raw_data[12] << 8 | mpu_raw_data[13]);

    gyro[0] = (float)gx_raw / GYRO_SENSITIVITY;
    gyro[1] = (float)gy_raw / GYRO_SENSITIVITY;
    gyro[2] = (float)gz_raw / GYRO_SENSITIVITY;
}
