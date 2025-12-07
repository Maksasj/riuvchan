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

#include "easings.h"

#include "components/bmp280.h"
#include "components/mpu6050.h"
#include "components/ssd1306.h"
#include "components/ttp223.h"

#include "engine/state_machine.h"
#include "engine/emotion_state.h"

void state_machine_update_task(void *ignore);

void ssd1306_display_render_task(void *ignore);
void bmp280_sensor_read_task(void *ignore);
void ttp223_sensor_read_task(void *ignore);
void mpu6050_sensor_read_task(void *ignore);

typedef struct face_state_t {
    uint8_t left_eye_x_position;
    uint8_t left_eye_y_position;
    uint8_t left_eye_width;
    uint8_t left_eye_height; 

    uint8_t right_eye_x_position;
    uint8_t right_eye_y_position;
    uint8_t right_eye_width;
    uint8_t right_eye_height; 

    uint8_t mouth_angle_start;
    uint8_t mouth_angle_end;
    uint8_t mouth_angle_x_position;
    uint8_t mouth_angle_y_position;
    uint8_t mouth_width;

    uint8_t decoration_width;
    uint8_t decoration_left_x_position;
    uint8_t decoration_left_y_position;

    uint8_t decoration_right_x_position;
    uint8_t decoration_right_y_position;
} face_state_t;

face_state_t* create_face_state(face_state_t state) {
    face_state_t* new_state = malloc(sizeof(face_state_t));
    *new_state = state;
    return new_state;
}

face_state_t calculate_face_state(face_state_t* current_face_state, face_state_t* previous_face_state, float transition);

typedef struct {
    i2c_config_t i2c;

    // Control flags for various tasks
    bool ttp223_read_loop_enabled;
    bool bmp280_read_loop_enabled;
    bool mpu605_read_loop_enabled;
    bool ssd1306_render_loop_enabled;

    face_state_t current_face_state;
    state_machine_t state_machine;
    emotion_state_t emotion_state;
} robot_state_t;

robot_state_t robot;

void init_robot_state(robot_state_t* robot);
void dispose_robot_state(robot_state_t* robot);
void init_i2c_master(i2c_config_t* conf);

void app_main() {
    init_robot_state(&robot);
    // scan();

    #ifndef APP_CPU_NUM
    #define APP_CPU_NUM PRO_CPU_NUM
    #endif

    xTaskCreatePinnedToCore(state_machine_update_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);

    xTaskCreatePinnedToCore(ttp223_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(bmp280_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    // xTaskCreatePinnedToCore(mpu6050_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ssd1306_display_render_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

void init_state_machine_states(state_machine_t* state_machine);

void init_robot_state(robot_state_t* robot) {
    // Initialize the emotion state
    init_emotion_state(&robot->emotion_state);

    // Initialize the state machine
    create_state_machine(&robot->state_machine);
    init_state_machine_states(&robot->state_machine);

    // Initialize flags to enable various tasks
    robot->ttp223_read_loop_enabled = true;
    robot->bmp280_read_loop_enabled = true;
    robot->mpu605_read_loop_enabled = true;
    robot->ssd1306_render_loop_enabled = true;

    // Initialize I2C Master Configuration
    init_i2c_master(&robot->i2c);
}

void init_state_machine_states(state_machine_t* state_machine) {
    add_state(&robot.state_machine, create_state("IDLE_STRAIGHT_EYES_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 5,
        .left_eye_width = 25,
        .left_eye_height = 40,

        .right_eye_x_position = SSD1306_WIDTH - 10 - 25,
        .right_eye_y_position = 5,
        .right_eye_width = 25,
        .right_eye_height = 40,

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = 64,
        .mouth_angle_y_position = 32,
        .mouth_width = 6,

        .decoration_width = 15,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 55,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 10,
        .decoration_right_y_position = 55
    })));

    add_state(&robot.state_machine, create_state("IDLE_EYES_RIGHT_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 5,
        .left_eye_width = 25,
        .left_eye_height = 40, 

        .right_eye_x_position = 10 + 25 + 5,
        .right_eye_y_position = 5,
        .right_eye_width = 25,
        .right_eye_height = 40, 

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = 10 + 25 + 2,
        .mouth_angle_y_position = 32,
        .mouth_width = 5,

        .decoration_width = 9,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 49,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 10,
        .decoration_right_y_position = 49
    })));

    add_state(&robot.state_machine, create_state("IDLE_EYES_LEFT_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = SSD1306_WIDTH - 10 - 25 - 5 - 25,
        .left_eye_y_position = 5,
        .left_eye_width = 25,
        .left_eye_height = 40, 

        .right_eye_x_position = SSD1306_WIDTH - 10 - 25,
        .right_eye_y_position = 5,
        .right_eye_width = 25,
        .right_eye_height = 40, 

        .mouth_angle_start = 60,
        .mouth_angle_end = 120,
        .mouth_angle_x_position = SSD1306_WIDTH - 10 - 25 - 2,
        .mouth_angle_y_position = 32,
        .mouth_width = 2,

        .decoration_width = 9,
        .decoration_left_x_position = SSD1306_WIDTH - 10 - 8 - 45,
        .decoration_left_y_position = 47,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 8,
        .decoration_right_y_position = 47
    })));

    add_state(&robot.state_machine, create_state("IDLE_SLEEP_STATE", create_face_state((face_state_t) {
        .left_eye_x_position = 10,
        .left_eye_y_position = 40,
        .left_eye_width = 30,
        .left_eye_height = 5,

        .right_eye_x_position = SSD1306_WIDTH - 10 - 25,
        .right_eye_y_position = 40,
        .right_eye_width = 30,
        .right_eye_height = 5,

        .mouth_angle_start = 80,
        .mouth_angle_end = 100,
        .mouth_angle_x_position = 64,
        .mouth_angle_y_position = 32,
        .mouth_width = 6,

        .decoration_width = 15,
        .decoration_left_x_position = 10,
        .decoration_left_y_position = 55,

        .decoration_right_x_position = SSD1306_WIDTH - 10 - 5,
        .decoration_right_y_position = 55
    })));

    robot.state_machine.current_state_index = 0;
    robot.state_machine.previous_state_index = 0;
}

void dispose_robot_state(robot_state_t* robot) {
    free_state_machine_states(&robot->state_machine);
    free_state_machine(&robot->state_machine);
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

void state_machine_update_task(void *ignore) {
    while (1) {
        float timer = 0.5f * sin(esp_timer_get_time() / 1000.0f) + 0.5f;

        // update_state_machine(&robot.state_machine, &robot.emotion_state);
        
        // Animation
        state_t* current_state = get_current_state(&robot.state_machine);
        state_t* previous_state = get_previous_state(&robot.state_machine);

        face_state_t* current_face_state = (face_state_t*) current_state->user_data;
        face_state_t* previous_face_state = NULL;

        if(previous_state != NULL)
            previous_face_state = (face_state_t*) previous_state->user_data;

        robot.current_face_state = calculate_face_state(current_face_state, previous_face_state, timer);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void render_eyes(uint8_t *screen_buffer, face_state_t* face_state) {
    // Right Eye    
    ssd1306_fill_round_rect(
        screen_buffer,
        face_state->right_eye_x_position,
        face_state->right_eye_y_position,
        face_state->right_eye_width,
        face_state->right_eye_height,
        8,
        1 
    );

    // Left Eye
    ssd1306_fill_round_rect(
        screen_buffer,
        face_state->left_eye_x_position,
        face_state->left_eye_y_position,
        face_state->left_eye_width,
        face_state->left_eye_height,
        8,
        1 
    );

    // ssd1306_draw_heart(screen_buffer, 22, 25, 30, 1);
    // ssd1306_draw_heart(screen_buffer, SSD1306_WIDTH - 30, 25, 30, 1);
}

void render_mouth(uint8_t *screen_buffer, face_state_t* face_state) {
    ssd1306_draw_arc(
        screen_buffer, 
        face_state->mouth_angle_x_position, 
        face_state->mouth_angle_y_position, 
        25, 
        face_state->mouth_angle_start, 
        face_state->mouth_angle_end, 
        face_state->mouth_width, 
        1
    ); 
}

void render_decorations(uint8_t *screen_buffer, face_state_t* face_state) {
    // uint8_t decoration_small = face_state->decoration_width / 3;
    // uint8_t decoration_big = decoration_small * 2;

    uint8_t decoration_small = 5;
    uint8_t decoration_big = 10;

    // Left
    ssd1306_fill_rect(
        screen_buffer, 
        face_state->decoration_left_x_position, 
        face_state->decoration_left_y_position, 
        decoration_big, 
        2, 
        1
    );

    ssd1306_fill_rect(
        screen_buffer, 
        face_state->decoration_left_x_position + decoration_big + 2 + 1, 
        face_state->decoration_left_y_position, 
        decoration_small, 
        2, 
        1
    );

    // Right
    ssd1306_fill_rect(
        screen_buffer, 
        face_state->decoration_right_x_position, 
        face_state->decoration_right_y_position, 
        decoration_big, 
        2, 
        1
    );

    ssd1306_fill_rect(
        screen_buffer, 
        face_state->decoration_right_x_position - decoration_big + 2, 
        face_state->decoration_right_y_position, 
        decoration_small, 
        2,
        1
    );
}

face_state_t calculate_face_state(face_state_t* current_face_state, face_state_t* previous_face_state, float transition) {
    if(previous_face_state == NULL)
        return *current_face_state;
        
    face_state_t face_state;

    face_state.left_eye_x_position = ease_in_out_cubic(previous_face_state->left_eye_x_position, current_face_state->left_eye_x_position, transition);
    face_state.left_eye_y_position = ease_in_out_cubic(previous_face_state->left_eye_y_position, current_face_state->left_eye_y_position, transition);
    face_state.left_eye_width = ease_in_out_cubic(previous_face_state->left_eye_width, current_face_state->left_eye_width, transition);
    face_state.left_eye_height = ease_in_out_cubic(previous_face_state->left_eye_height, current_face_state->left_eye_height, transition); 

    face_state.right_eye_x_position = ease_in_out_cubic(previous_face_state->right_eye_x_position, current_face_state->right_eye_x_position, transition);
    face_state.right_eye_y_position = ease_in_out_cubic(previous_face_state->right_eye_y_position, current_face_state->right_eye_y_position, transition);
    face_state.right_eye_width = ease_in_out_cubic(previous_face_state->right_eye_width, current_face_state->right_eye_width, transition);
    face_state.right_eye_height = ease_in_out_cubic(previous_face_state->right_eye_height, current_face_state->right_eye_height, transition);

    face_state.mouth_angle_start = ease_in_out_cubic(previous_face_state->mouth_angle_start, current_face_state->mouth_angle_start, transition);
    face_state.mouth_angle_end = ease_in_out_cubic(previous_face_state->mouth_angle_end, current_face_state->mouth_angle_end, transition);
    face_state.mouth_angle_x_position = ease_in_out_cubic(previous_face_state->mouth_angle_x_position, current_face_state->mouth_angle_x_position, transition);
    face_state.mouth_angle_y_position = ease_in_out_cubic(previous_face_state->mouth_angle_y_position, current_face_state->mouth_angle_y_position, transition);
    face_state.mouth_width = ease_in_out_cubic(previous_face_state->mouth_width, current_face_state->mouth_width, transition);


    face_state.decoration_width = ease_in_out_cubic(previous_face_state->decoration_width, current_face_state->decoration_width, transition);
    face_state.decoration_left_x_position = ease_in_out_cubic(previous_face_state->decoration_left_x_position, current_face_state->decoration_left_x_position, transition);
    face_state.decoration_left_y_position = ease_in_out_cubic(previous_face_state->decoration_left_y_position, current_face_state->decoration_left_y_position, transition);

    face_state.decoration_right_x_position = ease_in_out_cubic(previous_face_state->decoration_right_x_position, current_face_state->decoration_right_x_position, transition);
    face_state.decoration_right_y_position = ease_in_out_cubic(previous_face_state->decoration_right_y_position, current_face_state->decoration_right_y_position, transition);

    return face_state;
}

void render_face(uint8_t *screen_buffer) {
    face_state_t face_state = robot.current_face_state;

    render_eyes(screen_buffer, &face_state);
    render_mouth(screen_buffer, &face_state);
    render_decorations(screen_buffer, &face_state);
}

void render_scene(uint8_t *screen_buffer) {
    render_face(screen_buffer);
}

void ssd1306_display_render_task(void *ignore) {
    ESP_LOGI(TAG, "Starting OLED SSD1306 Demo");

    // Initialize the SSD1306 Display
    if (ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 initialization failed! Check wiring/address (0x3C).");
        vTaskDelete(NULL);
        return;
    }

    uint8_t *screen_buffer = malloc(SSD1306_WIDTH * SSD1306_HEIGHT / 8);

    while(robot.ssd1306_render_loop_enabled) {
        memset(screen_buffer, 0, SSD1306_WIDTH * SSD1306_HEIGHT / 8);

        render_scene(screen_buffer);
        ssd1306_draw_buffer(screen_buffer);
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
    while(robot.bmp280_read_loop_enabled) {
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

        if (touch_level == 1) {
            robot.state_machine.current_state_index = 0;
            robot.state_machine.previous_state_index = 0;
            vTaskDelay(pdMS_TO_TICKS(20));
            robot.state_machine.current_state_index = 3;
            robot.state_machine.previous_state_index = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
            robot.state_machine.current_state_index = 3;
            robot.state_machine.previous_state_index = 3;
            vTaskDelay(pdMS_TO_TICKS(5000));

            // ESP_LOGI(TAG, "SENSOR STATUS: >>> TOUCH DETECTED! <<<");
        } else {
            // ESP_LOGI(TAG, "SENSOR STATUS: Not Touched (LOW)");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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
    while(robot.mpu605_read_loop_enabled) {
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