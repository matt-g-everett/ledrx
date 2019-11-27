#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "ws2811.h"
#include "led.h"
#include "pixels.h"

const static char *TAG = "LED";

FRAME_t frame_buffer[CONFIG_LED_FRAME_BUFFER_SIZE];

uint8_t _running = 0;

void led_initialise(int gpio_num) {
    _running = 1;
    ws2811_init(gpio_num);
}

void led_set_running(uint8_t running) {
    ESP_LOGI(TAG, "Setting LED state to %s.", running ? "running" : "stopped");
    _running = running;
}

void led_task(void *pParam) {
    while(true) {
        if (_running) {
            ws2811_setColors(CONFIG_LED_NUM_PIXELS, frame_buffer[0].data);
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
        else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
