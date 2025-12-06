#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SDA_PIN 5
#define SCL_PIN 6

static const char *TAG = "i2cscanner";

#include "sht3x.h" // Component header

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

#define I2C_MASTER_NUM 0

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

// --- SSD1306 Commands and Constants ---
#define SSD1306_WIDTH          128
#define SSD1306_HEIGHT         64
#define SSD1306_CMD_MODE       0x00     // Co = 0, D/C = 0
#define SSD1306_DATA_MODE      0x40     // Co = 0, D/C = 1

// Full list of initialization commands for a 128x64 display
static const uint8_t init_cmds[] = {
    0xAE,          // 0xAE: Display OFF
    0xD5, 0x80,    // 0xD5: Set Display Clock Divide Ratio / Oscillator Frequency
    0xA8, 0x3F,    // 0xA8: Set Multiplex Ratio (63 for 64 height)
    0xD3, 0x00,    // 0xD3: Set Display Offset (no offset)
    0x40,          // 0x40: Set Start Line (line 0)
    0x8D, 0x14,    // 0x8D: Charge Pump Setting (VCC generated by internal DC/DC)
    0x20, 0x00,    // 0x20: Set Memory Addressing Mode (Horizontal Addressing Mode)
    0xA1,          // 0xA1: Set Segment Re-map (Horizontal flip)
    0xC8,          // 0xC8: Set COM Output Scan Direction (Vertical flip)
    0xDA, 0x12,    // 0xDA: Set COM Pins Hardware Configuration
    0x81, 0xCF,    // 0x81: Set Contrast Control
    0xD9, 0xF1,    // 0xD9: Set Pre-charge Period
    0xDB, 0x40,    // 0xDB: Set VCOMH Deselect Level
    0xA4,          // 0xA4: Entire Display ON (A5: All pixels ON)
    0xA6,          // 0xA6: Normal Display (A7: Inverse Display)
    0x2E,          // 0x2E: Deactivate Scroll
    0xAF           // 0xAF: Display ON
};

// Simple 8x8 font data for 'H', 'E', 'L', 'O', ',', ' ', 'E', 'S', 'P', '3', '2', '!'
// Each character is 8 bytes (1 page column)

#include "font.h"

// Helper function to map ASCII characters to the custom font array index
int char_to_font_index(char c) {
    return c; // Default to space
}

#define SSD1306_I2C_ADDR 0x3C

/**
 * @brief Send a command to the SSD1306.
 * The command byte is sent right after the Control Byte (0x00 for command mode).
 */
static esp_err_t ssd1306_send_command(uint8_t command) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Address + Write Bit
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDR << 1) | I2C_MASTER_WRITE, true); 
    
    // Control Byte: Command Mode (0x00)
    i2c_master_write_byte(cmd, SSD1306_CMD_MODE, true);
    
    // Command Byte
    i2c_master_write_byte(cmd, command, true);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Send a block of data to the SSD1306 GDDRAM.
 * The data block is preceded by the Control Byte (0x40 for data mode).
 */
static esp_err_t ssd1306_send_data(const uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Address + Write Bit
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    
    // Control Byte: Data Mode (0x40)
    i2c_master_write_byte(cmd, SSD1306_DATA_MODE, true);
    
    // Data Block
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief Initialize the SSD1306 with the necessary command sequence.
 */
static esp_err_t ssd1306_init(void) {
    ESP_LOGI(TAG, "Starting SSD1306 initialization...");
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < sizeof(init_cmds); i++) {
        // Some commands are two bytes long (command + argument), 
        // they are already grouped in the init_cmds array.
        if (ret != ESP_OK) break;
        ret = ssd1306_send_command(init_cmds[i]);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SSD1306 initialized successfully.");
    } else {
        ESP_LOGE(TAG, "SSD1306 initialization failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Clears the entire GDDRAM by sending zeroed data to all pages.
 */
static esp_err_t ssd1306_clear(void) {
    esp_err_t ret = ESP_OK;
    
    // The screen is 8 pages high (64 pixels / 8 bits per page = 8 pages)
    for (uint8_t page = 0; page < (SSD1306_HEIGHT / 8); page++) {
        // 1. Set Page Address (0xB0 | page_num)
        ret = ssd1306_send_command(0xB0 | page); 
        if (ret != ESP_OK) return ret;

        // 2. Set Lower Column Start Address (0x00)
        ret = ssd1306_send_command(0x00);
        if (ret != ESP_OK) return ret;

        // 3. Set Higher Column Start Address (0x10)
        ret = ssd1306_send_command(0x10);
        if (ret != ESP_OK) return ret;
        
        // Create an array of 128 zero bytes for one page
        uint8_t clear_page_data[SSD1306_WIDTH] = {0}; 
        
        // 4. Send the 128 zero bytes to clear the page
        ret = ssd1306_send_data(clear_page_data, SSD1306_WIDTH);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

/**
 * @brief Writes a string to the display starting at a specific page (row) and column.
 * It uses the simple 8x8 font defined above.
 * @param str The string to display.
 * @param page The starting page (0-7 for 64-pixel height).
 * @param col The starting column (0-127).
 */
static esp_err_t ssd1306_write_text(const char *str, uint8_t page, uint8_t col) {
    esp_err_t ret = ESP_OK;
    if (page >= (SSD1306_HEIGHT / 8)) return ESP_FAIL; // Invalid page

    // 1. Set Page Address (row)
    ret = ssd1306_send_command(0xB0 | page); 
    if (ret != ESP_OK) return ret;

    // 2. Set Column Address (col)
    // The column address is split into lower (0x00-0x0F) and higher (0x10-0x1F) nibbles
    ret = ssd1306_send_command(0x00 | (col & 0x0F));     // Lower nibble of column start address
    if (ret != ESP_OK) return ret;
    ret = ssd1306_send_command(0x10 | (col >> 4));      // Higher nibble of column start address
    if (ret != ESP_OK) return ret;

    // 3. Write data for each character
    for (const char *p = str; *p; p++) {
        int index = char_to_font_index(*p);
        const uint8_t *char_data = font8x8[index];
        // Each character is 8 bytes of data (one column in GDDRAM)
        ret = ssd1306_send_data(char_data, 8); 
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

void draw(void *ignore) {
    ESP_LOGI(TAG, "Starting OLED SSD1306 Demo");

    // 2. Initialize the SSD1306 Display
    if (ssd1306_init() != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 initialization failed! Check wiring/address (0x3C).");
        return;
    }
   
    while(1) {
         // 3. Clear the display (black screen)
        if (ssd1306_clear() != ESP_OK) {
            ESP_LOGE(TAG, "SSD1306 clear failed!");
        }
    
        vTaskDelay(pdMS_TO_TICKS(100));

        ssd1306_write_text("HELLO", 2, 10);

        // Loop indefinitely to keep the display running
        vTaskDelay(pdMS_TO_TICKS(1000));
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




void gyro(void *ignore) {
    /*
    // 1. Initialize the MPU-9250 (main chip only)
    if (mpu9250_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU-9250 6-axis initialization failed! Deleting task.");
        vTaskDelete(NULL); 
        return; 
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t mpu_raw_data[MPU9250_DATA_LEN]; // Accel/Gyro
    float accel[3], gyro[3]; 
    
    // 2. Main reading loop
    while(1) {
        esp_err_t mpu_err = mpu_read_data(mpu_raw_data);
        
        if (mpu_err == ESP_OK) {
            mpu9250_process_data(mpu_raw_data, accel, gyro);
            
            // Log 6-axis data
            printf("Accel(g): X:% 7.3f | Y:% 7.3f | Z:% 7.3f   ", accel[0], accel[1], accel[2]);
            printf("Gyro(d/s): X:% 7.3f | Y:% 7.3f | Z:% 7.3f\n", gyro[0], gyro[1], gyro[2]);
            
        } else {
            // Log specific error details when the MPU read fails
            ESP_LOGE(TAG, "MPU (Accel/Gyro) read failed (Code: %d)!", mpu_err);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Read data 10 times per second
    }
    vTaskDelete(NULL); 
*/
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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#define TOUCH_WAKEUP_GPIO   GPIO_NUM_0

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