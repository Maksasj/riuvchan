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

#include "utils.h"

#include "components/bmp280.h"
#include "components/mpu6050.h"
#include "components/ssd1306.h"
#include "components/ttp223.h"

void ssd1306_display_render_task(void *ignore);
void bmp280_sensor_read_task(void *ignore);
void ttp223_sensor_read_task(void *ignore);
void mpu6050_sensor_read_task(void *ignore);

typedef struct {
    i2c_config_t i2c;

    bool ttp223_read_loop_enabled;

} robot_state_t;

robot_state_t robot;

void init_robot_state(robot_state_t* robot);
void init_i2c_master(i2c_config_t* conf);

void app_main() {
    init_robot_state(&robot);
    // scan();

    #ifndef APP_CPU_NUM
    #define APP_CPU_NUM PRO_CPU_NUM
    #endif

    xTaskCreatePinnedToCore(ttp223_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(bmp280_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(mpu6050_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ssd1306_display_render_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

void init_robot_state(robot_state_t* robot) {
    robot->ttp223_read_loop_enabled = true;

    // Initialize I2C Master Configuration
    init_i2c_master(&robot->i2c);
}

void init_i2c_master(i2c_config_t* conf) {
    #define SDA_PIN 5
    #define SCL_PIN 6

    conf->mode = I2C_MODE_MASTER;
    conf->sda_io_num = SDA_PIN;
    conf->scl_io_num = SCL_PIN;
    conf->sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf->scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf->master.clk_speed = 100000;
    i2c_param_config(I2C_NUM_0, conf);

    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void ssd1306_display_render_task(void *ignore) {
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

void bmp280_sensor_read_task(void *ignore) {
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

void ttp223_sensor_read_task(void *ignore) {
    ttp223_t sensor;
    ttp223_init(&sensor, 0);

    while (robot.ttp223_read_loop_enabled) {
        int touch_level = ttp223_touched(&sensor);

        if (touch_level == 1)
            ESP_LOGI(TAG, "SENSOR STATUS: >>> TOUCH DETECTED! <<<");
        else
            ESP_LOGI(TAG, "SENSOR STATUS: Not Touched (LOW)");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void mpu6050_sensor_read_task(void *ignore) {
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