#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"

#include "ws2811.h"
#include "led.h"
#include "pixels.h"

const static char *TAG = "LED";

FRAME_t _frame_buffer[CONFIG_LED_FRAME_BUFFER_SIZE];

bool _log = false;
bool _log_peek_empty = false;
bool _force_log_drop = false;
uint8_t _seq = 0;
led_log _log_callback = NULL;
led_ack _ack_callback = NULL;
uint8_t _running = 0;
uint8_t _head = 0;
uint8_t _tail = 0;

uint16_t _dropCount = 0;
int64_t _sampling_start = 0;

// reads a byte from the buffer and return ERROR_EMPTY if buffer empty
static FRAME_t * fifo_peek() {
    if (_head == _tail) {
        if (_log && _log_peek_empty) {
            char msg[60];
            sprintf(msg, "BUF P EMPTY (H:%d, T:%d, S:%d)", _head, _tail, _seq);
            _log_callback(msg);
            _seq++;
        }

        //ESP_LOGI(TAG, "fifo_peek failed (H: %d, T: %d, S:%d)", _head, _tail, _seq);
        return NULL;
    }

    uint8_t peek_index = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    if (_log) {
        char msg[60];
        sprintf(msg, "BUF P %d (A:%d, H:%d, T:%d, S:%d)", peek_index, _frame_buffer[peek_index].ackID, _head, _tail, _seq);
        _log_callback(msg);
        _seq++;
    }
    return _frame_buffer + peek_index;
}

// reads a byte from the buffer and return 0 if buffer empty
static FRAME_t * fifo_read() {
    if (_head == _tail) {
        if (_log) {
            char msg[60];
            sprintf(msg, "BUF R EMPTY (H:%d, T:%d, S:%d)", _head, _tail, _seq);
            _log_callback(msg);
            _seq++;
        }
        return NULL;
    }

    _tail = (_tail + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    if (_log) {
        char msg[60];
        sprintf(msg, "BUF R %d (A:%d, H:%d, T:%d, S:%d)", _tail, _frame_buffer[_tail].ackID, _head, _tail, _seq);
        _log_callback(msg);
        _seq++;
    }
    return _frame_buffer + _tail;
}

// writes a byte to the buffer if not ERROR_FULL
static uint8_t fifo_write(FRAME_t *frame) {
    uint8_t next_head = (_head + 1) % CONFIG_LED_FRAME_BUFFER_SIZE;
    if (next_head == _tail) {
        if (_force_log_drop || _log) {
            char msg[60];
            sprintf(msg, "BUF DROP (A:%d, H:%d, T:%d, S:%d)", frame->ackID, _head, _tail, _seq);
            _log_callback(msg);
            _seq++;
        }
        //ESP_LOGI(TAG, "************ DROPPED FRAME (%s, H:%d, T:%d) ************", _running ? "running" : "stopped", _head, _tail);
        _dropCount++;
        return false;
    }
    _frame_buffer[next_head] = *frame;
    _head = next_head;
    if (_log) {
        char msg[60];
        sprintf(msg, "BUF W %d (A:%d, H:%d, T:%d, S:%d)", _head, frame->ackID, _head, _tail, _seq);
        _log_callback(msg);
        _seq++;
    }
    return true;
}

void led_initialise(led_log log_callback, led_ack ack_callback, int *gpios, size_t count) {
    _running = true;
    _log_callback = log_callback;
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
    int64_t delta;

    _sampling_start = esp_timer_get_time();

    while(true) {
        delta = esp_timer_get_time() - _sampling_start;
        if (delta > 1000000) {
            float fps = (float)_dropCount / ((float)delta / 1000000.0f);
            char msg[60];
            sprintf(msg, "BUF DROP FPS %.1f (S:%d)", fps, _seq);
            _log_callback(msg);
            _seq++;

            // Reset sampling period
            _dropCount = 0;
            _sampling_start = esp_timer_get_time();
        }

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
