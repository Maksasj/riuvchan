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

#define SHT30_CMD_MEAS_HIGH_REP_MSB 0x24
#define SHT30_CMD_MEAS_HIGH_REP_LSB 0x00

#define SHT30_SENSOR_ADDR 0x44

esp_err_t sht30_send_command(i2c_port_t i2c_num, uint8_t command_msb, uint8_t command_lsb) {
    uint8_t write_buf[2] = {command_msb, command_lsb};
    // i2c_master_write_to_device is a simplified wrapper for sending data
    return i2c_master_write_to_device(i2c_num, SHT30_SENSOR_ADDR, write_buf, 2, 1000 / portTICK_PERIOD_MS);
}

#define SHT30_READ_LEN 6

esp_err_t sht30_read_data(i2c_port_t i2c_num, uint8_t *data_buf) {
    // Read 6 bytes directly from the sensor's address
    return i2c_master_read_from_device(i2c_num, SHT30_SENSOR_ADDR, data_buf, SHT30_READ_LEN, 1000 / portTICK_PERIOD_MS);
}

void sht30_process_data(const uint8_t *data_buf, float *temperature, float *humidity) {
    // 1. Combine 8-bit bytes into 16-bit raw counts
    uint16_t temp_raw = (data_buf[0] << 8) | data_buf[1];
    uint16_t hum_raw = (data_buf[3] << 8) | data_buf[4];

    // Note: data_buf[2] and data_buf[5] are CRC checksums

    // 2. Apply the conversion formulas
    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = 100.0f * ((float)hum_raw / 65535.0f);
}