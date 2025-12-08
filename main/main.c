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
#include "states.h"

#include "easings.h"
#include "face_state.h"

#define HAUL_IMPLEMENTATION
#include "safe_vector.h"

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

typedef struct {
    i2c_config_t i2c;

    // Control flags for various tasks
    bool ttp223_read_loop_enabled;
    bool bmp280_read_loop_enabled;
    bool mpu605_read_loop_enabled;
    bool ssd1306_render_loop_enabled;

    safe_vector_t events;

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
    xTaskCreatePinnedToCore(bmp280_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(mpu6050_sensor_read_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ssd1306_display_render_task, TAG, configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}

void fire_event(event_t* event) {
    vector_push(&robot.events, event);
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

    create_vector(&robot->events, 128);
}

void init_state_machine_states(state_machine_t* state_machine) {
    add_all_states(state_machine);

    robot.state_machine.current_state_index = 0;
    robot.state_machine.previous_state_index = 0;
}

void dispose_robot_state(robot_state_t* robot) {
    free_state_machine_states(&robot->state_machine);
    free_state_machine(&robot->state_machine);

    free_vector_content(&robot->events);
    free_vector(&robot->events);
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

#include "utils.h"

void state_machine_update_task(void *ignore) {
    state_t* start_state = get_current_state(&robot.state_machine);
    robot.current_face_state = calculate_face_state(start_state, start_state, 0.0f);

    // transition
    float transition_start_time = esp_timer_get_time() / 1000.0f; 
    float transition_duration = 1.0f;
    while (1) {
        update_state_machine(&robot.state_machine, &robot.emotion_state);

        int event_count = vector_size(&robot.events);
        
        bool state_changed = false;
        if(event_count > 1) {
            event_t* event = vector_pop(&robot.events);

            if(event->type == TOUCH_TAP) {
                robot.emotion_state.happiness += 1;
                
                if(robot.emotion_state.anger > 50)
                    robot.emotion_state.anger -= 1;
            }

            if(event->type == TOUCH_MULTI_TAP && event->as.multi_tap >= 30) {
                if(robot.emotion_state.anger > 50)
                    robot.emotion_state.anger -= 30;
            }

            if(event->type == TOUCH_LONG_TAP) {
                robot.emotion_state.happiness += 10;

                if(robot.emotion_state.anger > 30)
                    robot.emotion_state.anger -= 10;
            }

            if(event->type == TOUCH_SUPER_LONG_TAP) {
                robot.emotion_state.happiness += 30;

                if(robot.emotion_state.anger > 30)
                    robot.emotion_state.anger -= 30;
            }

            if(event->type == THROWN_UP) {
                if(robot.emotion_state.happiness > 30)
                    robot.emotion_state.happiness -= 5;

                robot.emotion_state.anger += 10;
            }

            ESP_LOGE(TAG, "MOOD: H %f, S %f, F %f, A %f, D %f, S %f", robot.emotion_state.happiness, robot.emotion_state.sadness, robot.emotion_state.fear, robot.emotion_state.anger, robot.emotion_state.disgust, robot.emotion_state.surprise);

            if(!state_changed && strcmp(get_current_state(&robot.state_machine)->state_name, "IDLE_STRAIGHT_EYES_STATE") == 0) {
                if(event->type == TOUCH_TAP) {
                    int current = robot.state_machine.current_state_index;
                    robot.state_machine.current_state_index = 2;
                    robot.state_machine.previous_state_index = current;
                    transition_start_time = esp_timer_get_time() / 1000.0f;
                    transition_duration = 0.15f;
                    state_changed = true;
                }
            }
            
            if(!state_changed && strcmp(get_current_state(&robot.state_machine)->state_name, "IDLE_SLEEP_STATE") == 0) {
                if(event->type == TOUCH_TAP) {
                    int current = robot.state_machine.current_state_index;
                    robot.state_machine.current_state_index = 0;
                    robot.state_machine.previous_state_index = current;
                    transition_start_time = esp_timer_get_time() / 1000.0f;
                    transition_duration = 0.15f;
                    state_changed = true;
                }
            }
        }

        if(!state_changed && strcmp(get_current_state(&robot.state_machine)->state_name, "CONFUSED_CLOSED_EYES_STATE") == 0) {
            if(robot.emotion_state.anger < 50.0f) {
                int current = robot.state_machine.current_state_index;
                robot.state_machine.current_state_index = 0;
                robot.state_machine.previous_state_index = current;
                transition_start_time = esp_timer_get_time() / 1000.0f;
                transition_duration = 5.0f;
                state_changed = true;
            }
        }
        
        if(!state_changed && strcmp(get_current_state(&robot.state_machine)->state_name, "IDLE_STRAIGHT_EYES_STATE") == 0) {
            if(robot.emotion_state.anger > 75.0f) {
                int current = robot.state_machine.current_state_index;
                robot.state_machine.current_state_index = 3;
                robot.state_machine.previous_state_index = current;
                transition_start_time = esp_timer_get_time() / 1000.0f;
                transition_duration = 2.0f;
                state_changed = true;
            }
        }
        
        if(!state_changed && strcmp(get_current_state(&robot.state_machine)->state_name, "IDLE_STRAIGHT_EYES_STATE") == 0) {
            float current_time = esp_timer_get_time() / 1000.0f;

            if((current_time - transition_start_time) > 5000.0f) {
                int current = robot.state_machine.current_state_index;
                robot.state_machine.current_state_index = 1;
                robot.state_machine.previous_state_index = current;
                transition_start_time = esp_timer_get_time() / 1000.0f;
                transition_duration = 0.7f;
                state_changed = true;
            }
        }
        
        if(!state_changed && strcmp(get_current_state(&robot.state_machine)->state_name, "IDLE_CLOSED_EYES_STATE") == 0) {
            float current_time = esp_timer_get_time() / 1000.0f;

            if((current_time - transition_start_time) > 100.0f) {
                int current = robot.state_machine.current_state_index;
                robot.state_machine.current_state_index = 0;
                robot.state_machine.previous_state_index = current;
                transition_start_time = esp_timer_get_time() / 1000.0f;
                transition_duration = 0.7f;
                state_changed = true;
            }
        }

        float current_time = esp_timer_get_time() / 1000.0f;
        float duration_aspect = clamp(0.0f, 1.0f, ((current_time - transition_start_time) / 1000.0f) / transition_duration);

        // Animation
        state_t* current_state = get_current_state(&robot.state_machine);
        state_t* previous_state = get_previous_state(&robot.state_machine);

        face_state_t* current_face_state = (face_state_t*) current_state->user_data;
        face_state_t* previous_face_state = NULL;

        if(previous_state != NULL)
            previous_face_state = (face_state_t*) previous_state->user_data;

        robot.current_face_state = calculate_face_state(current_face_state, previous_face_state, duration_aspect);

        vTaskDelay(pdMS_TO_TICKS(16));
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
        vTaskDelay(pdMS_TO_TICKS(20));
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

            if(temp_c < 20.0f)
                robot.emotion_state.anger += 5.0f;

            if(temp_c > 25.0f) {
                robot.emotion_state.anger -= 10.0f;
                robot.emotion_state.happiness += 5.0f;
            }
        } else {
            ESP_LOGE(TAG, "BMP280 raw data read failed!");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Read data once per second
    }
    vTaskDelete(NULL);
}

// --- Config ---
#define DOUBLE_TAP_DELAY_US    300000   // 300ms: Window to wait for a second tap
#define HOLD_THRESHOLD_US      800000   // 0.8s: Time to trigger Hold
#define SUPER_HOLD_THRESHOLD_US 3000000 // 3.0s: Time to trigger Super Hold

typedef struct {
    int64_t press_start_time;
    int64_t release_time;
    int tap_count;
    bool hold_triggered;         // True if we already fired a Hold event during this press
    bool super_hold_triggered;   // True if we already fired a Super Hold
    bool last_level;
} button_context_t;

void ttp223_sensor_read_task(void *ignore) {
    ttp223_t sensor;
    ttp223_init(&sensor, 0);

    button_context_t btn = {0};
    btn.last_level = 0;

    ESP_LOGI("TOUCH", "Task Started (No Dead Zones)");

    while (robot.ttp223_read_loop_enabled) {
        bool current_level = ttp223_touched(&sensor);
        int64_t now = esp_timer_get_time();
        
        bool rising_edge  = (current_level && !btn.last_level);
        bool falling_edge = (!current_level && btn.last_level);
        btn.last_level    = current_level;

        // --- 1. Pressed Down ---
        if (rising_edge) {
            btn.press_start_time = now;
            btn.hold_triggered = false;
            btn.super_hold_triggered = false;
        }

        // --- 2. While Holding Down ---
        if (current_level) {
            int64_t duration = now - btn.press_start_time;

            // Trigger Super Hold
            if (duration > SUPER_HOLD_THRESHOLD_US && !btn.super_hold_triggered) {
                ESP_LOGI("TOUCH", ">>> SUPER LONG PRESS <<<");
                fire_event(create_touch_event(TOUCH_SUPER_LONG_TAP, -1));
                btn.super_hold_triggered = true;
                // Note: We don't need to reset tap_count here because 'hold_triggered' 
                // is already true, so the release logic below won't count it.
            }
            // Trigger Normal Hold
            else if (duration > HOLD_THRESHOLD_US && !btn.hold_triggered) {
                fire_event(create_touch_event(TOUCH_LONG_TAP, -1));
                ESP_LOGI("TOUCH", ">>> LONG PRESS (HOLD) <<<");
                btn.hold_triggered = true;
            }
        }

        // --- 3. Released ---
        if (falling_edge) {
            btn.release_time = now;

            // KEY FIX: Only count as a tap if we NEVER triggered a hold.
            // This eliminates the dead zone. If you release before 800ms, it is ALWAYS a tap.
            if (!btn.hold_triggered) {
                btn.tap_count++;
            }
        }

        // --- 4. Resolve Taps (While Released) ---
        if (!current_level && btn.tap_count > 0) {
            int64_t time_since_release = now - btn.release_time;

            if (time_since_release > DOUBLE_TAP_DELAY_US) {
                if (btn.tap_count == 1) {
                    fire_event(create_touch_event(TOUCH_TAP, -1));
                    ESP_LOGI("TOUCH", ">>> SINGLE TAP <<<");
                } 
                else if (btn.tap_count == 2) {
                    fire_event(create_touch_event(TOUCH_DOUBLE_TAP, -1));
                    ESP_LOGI("TOUCH", ">>> DOUBLE TAP <<<");
                }
                else {
                    fire_event(create_touch_event(TOUCH_MULTI_TAP, btn.tap_count));
                    ESP_LOGI("TOUCH", ">>> MULTI TAP (%d) <<<", btn.tap_count);
                }
                btn.tap_count = 0; // Reset
            }
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
            
            float ax = accel[0];
            float ay = accel[1];
            float az = accel[2]; 

            float gx = gyro[0];
            float gy = gyro[1];
            float gz = gyro[2]; 

            if(gx * gx + gy * gy + gz * gz >= 180.0f * 180.0f && ax * ax + ay * ay + az * az >= 2.0f * 2.0f) {
                fire_event(create_throw_up_event());
                ESP_LOGE(TAG, "Firing thrown event");
            }
        } else {
            ESP_LOGE(TAG, "Failed to read MPU-6050 data!");
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Read data 10 times per second
    }
}