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

#define MPU6050_I2C_ADDR     0x68       /*!< MPU-6050 default I2C address (AD0 low) */
#define MPU6050_PWR_MGMT_1   0x6B       /*!< Power Management 1 register */
#define MPU6050_ACCEL_XOUT_H 0x3B       /*!< Starting register address for 14 bytes of data */
#define MPU6050_DATA_LEN     14         /*!< Total bytes to read (Accel, Temp, Gyro) */

// Scale factors for default settings (Accel: +/- 2g, Gyro: +/- 250 deg/s)
#define ACCEL_SENSITIVITY    16384.0f   /*!< LSB per g */
#define GYRO_SENSITIVITY     131.0f     /*!< LSB per deg/s */
/**
 * @brief Writes a single byte to a register on the MPU-6050.
 */
static esp_err_t mpu6050_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR, write_buf, 2, 100 / portTICK_PERIOD_MS);
}

/**
 * @brief Initializes the MPU-6050.
 * Wakes the sensor up by writing 0x00 to the PWR_MGMT_1 register.
 */
static esp_err_t mpu6050_init(void) {
    ESP_LOGI(TAG, "Initializing MPU-6050...");
    // Wake up MPU-6050 (set SLEEP bit to 0)
    return mpu6050_write_byte(MPU6050_PWR_MGMT_1, 0x00);
}

/**
 * @brief Reads 14 bytes of raw data (Accel, Temp, Gyro) from the MPU-6050.
 * The MPU-6050 automatically increments the register address after the first byte is read.
 */
static esp_err_t mpu6050_read_data(uint8_t *data_buf) {
    // We first write the register address we want to start reading from (0x3B),
    // and then immediately read 14 bytes (MPU6050_DATA_LEN) starting from that address.
    uint8_t reg_addr = MPU6050_ACCEL_XOUT_H;
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR, &reg_addr, 1, data_buf, MPU6050_DATA_LEN, 100 / portTICK_PERIOD_MS);
}

/**
 * @brief Converts raw 16-bit MPU-6050 data into physical units.
 * Raw data is Big-Endian (MSB first).
 * * @param raw_data 14 bytes array read from MPU-6050.
 * @param accel Array to store 3 float accelerometer values (g).
 * @param gyro Array to store 3 float gyroscope values (deg/s).
 */
static void mpu6050_process_data(const uint8_t *raw_data, float *accel, float *gyro) {
    // --- Accelerometer Data (Bytes 0-5) ---
    // Ax (0, 1), Ay (2, 3), Az (4, 5)
    int16_t ax_raw = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    int16_t ay_raw = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    int16_t az_raw = (int16_t)(raw_data[4] << 8 | raw_data[5]);
    
    // Convert raw counts to g's
    accel[0] = (float)ax_raw / ACCEL_SENSITIVITY;
    accel[1] = (float)ay_raw / ACCEL_SENSITIVITY;
    accel[2] = (float)az_raw / ACCEL_SENSITIVITY;

    // --- Gyroscope Data (Bytes 8-13) ---
    // Gx (8, 9), Gy (10, 11), Gz (12, 13)
    int16_t gx_raw = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    int16_t gy_raw = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    int16_t gz_raw = (int16_t)(raw_data[12] << 8 | raw_data[13]);

    // Convert raw counts to deg/s
    gyro[0] = (float)gx_raw / GYRO_SENSITIVITY;
    gyro[1] = (float)gy_raw / GYRO_SENSITIVITY;
    gyro[2] = (float)gz_raw / GYRO_SENSITIVITY;
}


