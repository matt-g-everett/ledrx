#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "ws2811.h"
#include "led.h"
#include "pixels.h"

const static char *TAG = "LED";

FRAME_t _frame_buffer[CONFIG_LED_FRAME_BUFFER_SIZE];

led_ack _ack_callback = NULL;
uint8_t _running = 0;
uint8_t _head = 0;
uint8_t _tail = 0;

// reads a byte from the buffer and return ERROR_EMPTY if buffer empty
static FRAME_t * fifo_peek() {
    if (_head == _tail) {
        ESP_LOGI(TAG, "fifo_peek failed (H: %d, T: %d)", _head, _tail);
        return NULL;
    }

    uint8_t peek_index = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    return _frame_buffer + peek_index;
}

// reads a byte from the buffer and return 0 if buffer empty
static FRAME_t * fifo_read() {
    if (_head == _tail) {
        return NULL;
    }

    _tail = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    return _frame_buffer + _tail;
}

// writes a byte to the buffer if not ERROR_FULL
static uint8_t fifo_write(FRAME_t *frame) {
    uint8_t next_head = (_head + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    if (next_head == _tail) {
        ESP_LOGI(TAG, "************ DROPPED FRAME (%s, H:%d, T:%d) ************", _running ? "running" : "stopped", _head, _tail);
        return false;
    }

    _frame_buffer[next_head] = *frame;
    _head = next_head;
    return true;
}

void led_initialise(led_ack ack_callback, int *gpios, size_t count) {
    _running = true;
    _ack_callback = ack_callback;
    ws2811_init(gpios, count);
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
                ws2811_setColors(frame->len, frame->data);
                _ack_callback(frame->ackID);

                fifo_read(); // Consume the frame
                vTaskDelay(0 / portTICK_PERIOD_MS);
            }
            else {
                vTaskDelay(0 / portTICK_PERIOD_MS);
            }
        }
        else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
