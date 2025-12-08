#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"

#include "ws2811.h"
#include "led.h"
#include "pixels.h"

const static char *TAG = "LED";

// Static frame buffer pool
static FRAME_t _frame_buffer[CONFIG_LED_FRAME_BUFFER_SIZE];

// Queue handles for buffer pool management
static QueueHandle_t _free_slots = NULL;    // Available buffer slots
static QueueHandle_t _ready_frames = NULL;  // Frames ready to display

// Diagnostic logging (MQTT-based remote diagnostics)
static bool _log = false;
static bool _force_log_drop = false;
static uint8_t _seq = 0;
static led_log _log_callback = NULL;

// Callbacks and state
static led_ack _ack_callback = NULL;
static uint8_t _running = 0;

// Drop statistics
static uint16_t _dropCount = 0;
static int64_t _sampling_start = 0;

void led_initialise(led_log log_callback, led_ack ack_callback, int *gpios, size_t count) {
    _running = true;
    _log_callback = log_callback;
    _ack_callback = ack_callback;

    // Create queues for buffer pool management
    _free_slots = xQueueCreate(CONFIG_LED_FRAME_BUFFER_SIZE, sizeof(FRAME_t*));
    _ready_frames = xQueueCreate(CONFIG_LED_FRAME_BUFFER_SIZE, sizeof(FRAME_t*));

    // Pre-populate free slots queue with pointers to buffer pool entries
    for (int i = 0; i < CONFIG_LED_FRAME_BUFFER_SIZE; i++) {
        FRAME_t *slot = &_frame_buffer[i];
        xQueueSend(_free_slots, &slot, 0);
    }

    ws2811_init(gpios, count);
}

void led_set_running(uint8_t running) {
    ESP_LOGI(TAG, "Setting LED state to %s.", running ? "running" : "stopped");
    _running = running;
}

uint8_t led_push_stream(char *data) {
    FRAME_t *incoming = (FRAME_t*)data;
    FRAME_t *slot;

    ESP_LOGD(TAG, "led_push_stream: ackID=%d, len=%d", incoming->ackID, incoming->len);

    // Non-blocking acquire of a free slot
    if (xQueueReceive(_free_slots, &slot, 0) == pdTRUE) {
        // Copy frame data into the buffer pool slot
        memcpy(slot, incoming, sizeof(FRAME_t));

        // Enqueue for consumption by led_task
        xQueueSend(_ready_frames, &slot, 0);

        if (_log) {
            char msg[60];
            sprintf(msg, "BUF W (A:%d, S:%d)", incoming->ackID, _seq);
            _log_callback(msg);
            _seq++;
        }
        return true;
    }

    // Buffer full - drop frame
    if (_force_log_drop || _log) {
        char msg[60];
        sprintf(msg, "BUF DROP (A:%d, S:%d)", incoming->ackID, _seq);
        _log_callback(msg);
        _seq++;
    }
    _dropCount++;
    return false;
}

void led_task(void *pParam) {
    FRAME_t *frame;
    int64_t delta;

    _sampling_start = esp_timer_get_time();

    while(true) {
        // Only log drop stats if there were actual drops
        delta = esp_timer_get_time() - _sampling_start;
        if (delta > 1000000) {
            if (_dropCount > 0) {
                float fps = (float)_dropCount / ((float)delta / 1000000.0f);
                char msg[60];
                sprintf(msg, "BUF DROP FPS %.1f (S:%d)", fps, _seq);
                _log_callback(msg);
                _seq++;
            }

            // Reset sampling period
            _dropCount = 0;
            _sampling_start = esp_timer_get_time();
        }

        if (_running) {
            // Block up to 10ms waiting for frames - eliminates busy-wait
            if (xQueueReceive(_ready_frames, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (_log) {
                    char msg[60];
                    sprintf(msg, "BUF R (A:%d, S:%d)", frame->ackID, _seq);
                    _log_callback(msg);
                    _seq++;
                }

                ws2811_setColors(frame->len, frame->data);
                _ack_callback(frame->ackID);

                // Return slot to the free pool
                xQueueSend(_free_slots, &frame, 0);

                vTaskDelay(1); // Brief delay between frames for system stability
            }
            // No frame available within timeout - loop continues
        }
        else {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
