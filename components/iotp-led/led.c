#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "ws2811.h"
#include "led.h"
#include "pixels.h"

const static char *TAG = "LED";

FRAME_t _frame_buffer[CONFIG_LED_FRAME_BUFFER_SIZE];

uint8_t _running = 0;
uint8_t _head = 0;
uint8_t _tail = 0;

// reads a byte from the buffer and return ERROR_EMPTY if buffer empty
static FRAME_t * fifo_read() {
   if (_head == _tail) return NULL;
   _tail = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
   return _frame_buffer + _tail;
}

// writes a byte to the buffer if not ERROR_FULL
static uint8_t fifo_write(FRAME_t *frame) {
   if (_head + 1 == _tail) return false;
   _head = (_head + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
   _frame_buffer[_head] = *frame;
   return true;
}

void led_initialise(int gpio_num) {
    _running = true;
    ws2811_init(gpio_num);
}

void led_set_running(uint8_t running) {
    ESP_LOGI(TAG, "Setting LED state to %s.", running ? "running" : "stopped");
    _running = running;
}

uint8_t led_push_stream(char *data) {
    FRAME_t *frame = (FRAME_t*)data;
    //return fifo_write(frame);
    ESP_LOGI(TAG, "led data: %d", frame->len);
    ESP_LOGI(TAG, "led data: #%02x%02x%02x, #%02x%02x%02x", frame->data[0].r, frame->data[0].g, frame->data[0].b,
        frame->data[1].r, frame->data[1].g, frame->data[1].b);
    return 0;
}

void led_task(void *pParam) {
    FRAME_t *frame;
    while(true) {
        if (_running) {
            frame = fifo_read();
            if (frame != NULL) {
                ESP_LOGI(TAG, "Nothing in buffer.");
            }
            else {
                ESP_LOGI(TAG, "Displaying frame.");
            }
            //ws2811_setColors(CONFIG_LED_NUM_PIXELS, frame->data);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
