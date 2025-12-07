#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "assets/face.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "components/bmp280.h"
#include "components/sh30.h"
#include "components/mpu6050.h"
#include "components/ssd1306.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SDA_PIN 5
#define SCL_PIN 6


void scan() {
    while (1)
    {
        esp_err_t res;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (uint8_t i = 3; i < 0x78; i++)
        {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
            i2c_master_stop(cmd);
    
            res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
            if (i % 16 == 0)
                printf("\n%.2x:", i);
            if (res == 0)
                printf(" %.2x", i);
            else
                printf(" --");
            i2c_cmd_link_delete(cmd);
        }
        printf("\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void bar() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // 1. Initialize the BMP280
    if (bmp280_init() != ESP_OK) {
        ESP_LOGE(TAG, "BMP280 initialization failed! Deleting task.");
        vTaskDelete(NULL); 
        return; 
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    int32_t raw_P, raw_T;
    int32_t compensated_T_i32;
    uint32_t compensated_P_u32;
    
    // 2. Main reading loop
    while(1) {
        if (bmp280_read_raw_data(&raw_P, &raw_T) == ESP_OK) {
            
            // Compensation must be done in this order: Temperature then Pressure
            compensated_T_i32 = bmp280_compensate_T_int32(raw_T);
            compensated_P_u32 = bmp280_compensate_P_int32(raw_P);
            
            float temp_c = (float)compensated_T_i32 / 100.0f; // Scale temperature to DegC
            float pressure_hpa = (float)compensated_P_u32 / 100.0f; // Scale Pa to hPa (mbar)

            // Log the compensated data
            printf("Temp(C): % 6.2f | Pressure(hPa): % 7.2f\n", temp_c, pressure_hpa);
            
        } else {
            ESP_LOGE(TAG, "BMP280 raw data read failed!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Read data once per second
    }
    vTaskDelete(NULL);
}

void temperature(void *ignore) {
    uint8_t data[SHT30_READ_LEN];
    float temp, hum;

    while (1) {
        // 1. Send the measurement command 0x2400
        if (sht30_send_command(I2C_MASTER_NUM, SHT30_CMD_MEAS_HIGH_REP_MSB, SHT30_CMD_MEAS_HIGH_REP_LSB) != ESP_OK) {
            ESP_LOGE("SHT30", "Failed to send command");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // 2. Wait for conversion
        vTaskDelay(pdMS_TO_TICKS(20));

        // 3. Read 6 bytes of data
        if (sht30_read_data(I2C_MASTER_NUM, data) != ESP_OK) {
            ESP_LOGE("SHT30", "Failed to read data");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // 4. Process and print
        sht30_process_data(data, &temp, &hum);
        
        printf("T: %.2f °C, RH: %.2f %%\n", temp, hum);

        vTaskDelay(pdMS_TO_TICKS(250)); // Read every 2 seconds
    }
}

void draw(void *ignore) {
    ESP_LOGI(TAG, "Starting OLED SSD1306 Demo");

    // Initialize the SSD1306 Display
    if (ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 initialization failed! Check wiring/address (0x3C).");
        vTaskDelete(NULL);
        return;
    }

    while(1) {
        ssd1306_draw_buffer(face_img);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#define TOUCH_SENSOR_GPIO 0

void touch(void *ignore) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE; // No interrupts needed for polling
    io_conf.mode = GPIO_MODE_INPUT;        // Set as input mode
    io_conf.pin_bit_mask = (1ULL << TOUCH_SENSOR_GPIO); // Bit mask for GPIO 14
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // TTP223 is open-drain output (active low/high), so no internal pull required
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    // Apply the configuration
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Touch Sensor Monitor initialized on GPIO %d.", TOUCH_SENSOR_GPIO);

    // 2. Continuous Monitoring Loop
    while (1) {
        // Read the state of the GPIO pin
        int touch_level = gpio_get_level(TOUCH_SENSOR_GPIO);

        if (touch_level == 1) {
            // TTP223 outputs HIGH when touched (if configured for Active-High)
            ESP_LOGI(TAG, "SENSOR STATUS: >>> TOUCH DETECTED! <<<");
        } else {
            // TTP223 outputs LOW when not touched
            ESP_LOGI(TAG, "SENSOR STATUS: Not Touched (LOW)");
        }

        // Delay for 500ms to prevent the watchdog timer from triggering
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void gyro(void *ignore) {
    // 2. Initialize the MPU-6050 Sensor
    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU-6050 initialization failed! Check wiring/address (0x68).");
        return;
    }
    ESP_LOGI(TAG, "MPU-6050 initialized successfully.");
    
    // Wait a moment for sensor stabilization
    vTaskDelay(pdMS_TO_TICKS(200));

    uint8_t raw_data[MPU6050_DATA_LEN];
    float accel[3]; // X, Y, Z acceleration in g
    float gyro[3];  // X, Y, Z angular velocity in deg/s
    
    // 3. Main reading loop
    while(1) {
        if (mpu6050_read_data(raw_data) == ESP_OK) {
            mpu6050_process_data(raw_data, accel, gyro);
            
            // Log formatted data to the console
            printf("Accel (g): X: % 7.3f | Y: % 7.3f | Z: % 7.3f   ", accel[0], accel[1], accel[2]);
            printf("Gyro (deg/s): X: % 7.3f | Y: % 7.3f | Z: % 7.3f\n", gyro[0], gyro[1], gyro[2]);
            
        } else {
            ESP_LOGE(TAG, "Failed to read MPU-6050 data!");
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Read data 10 times per second
    }
}

i2c_config_t conf;
    
void app_main() {
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, &conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    
    // scan();

    xTaskCreatePinnedToCore(touch, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(temperature, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(draw, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(gyro, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(bar, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}