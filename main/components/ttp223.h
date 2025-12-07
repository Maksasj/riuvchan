#ifndef TTP223_H
#define TTP223_H

typedef struct {
    uint64_t pin;
} ttp223_t;

static esp_err_t ttp223_init(ttp223_t *sensor, uint64_t pin) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    sensor->pin = pin;
    
    ESP_LOGI(TAG, "Touch Sensor Monitor initialized on GPIO %d.", 0);
    
    return gpio_config(&io_conf);
}

static int ttp223_touched(ttp223_t *sensor) {
    return gpio_get_level(sensor->pin);
}

#endif
