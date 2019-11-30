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
static FRAME_t * fifo_peek() {
    // ESP_LOGI(TAG, "fifo_peek enter - head: %d, tail: %d", _head, _tail);
    if (_head == _tail) {
        ESP_LOGI(TAG, "fifo_peek failed (H: %d, T: %d)", _head, _tail);
        return NULL;
    }

    // ESP_LOGI(TAG, "fifo_peek exit - head: %d, tail: %d", _head, _tail);
    uint8_t peek_index = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    return _frame_buffer + peek_index;
}

// reads a byte from the buffer and return 0 if buffer empty
static FRAME_t * fifo_read() {
    // ESP_LOGI(TAG, "fifo_read enter - head: %d, tail: %d", _head, _tail);
    if (_head == _tail) {
        return NULL;
    }

    _tail = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    // ESP_LOGI(TAG, "fifo_read exit - head: %d, tail: %d", _head, _tail);
    return _frame_buffer + _tail;
}

// writes a byte to the buffer if not ERROR_FULL
static uint8_t fifo_write(FRAME_t *frame) {
    // ESP_LOGI(TAG, "fifo_write enter - head: %d, tail: %d", _head, _tail);
    uint8_t next_head = (_head + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    if (next_head == _tail) {
        ESP_LOGI(TAG, "************ DROPPED FRAME (%s, H:%d, T:%d) ************", _running ? "running" : "stopped", _head, _tail);
        return false;
    }

    _frame_buffer[next_head] = *frame;
    _head = next_head;
    // ESP_LOGI(TAG, "fifo_write exit - head: %d, tail: %d", _head, _tail);
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
    return fifo_write((FRAME_t*)data);
}

void led_task(void *pParam) {
    FRAME_t *frame;
    while(true) {
        if (_running) {
            frame = fifo_peek(); // Only peek the frame so the memory doesn't get overwritten
            if (frame != NULL) {
                // ESP_LOGI(TAG, "led data: %d", frame->len);
                // ESP_LOGI(TAG, "led data: #%02x%02x%02x, #%02x%02x%02x", frame->data[0].r, frame->data[0].g, frame->data[0].b,
                //     frame->data[1].r, frame->data[1].g, frame->data[1].b);
                ws2811_setColors(frame->len, frame->data);
                fifo_read(); // Consume the frame
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
            else {
                // ESP_LOGI(TAG, "Nothing in buffer.");
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
        }
        else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
