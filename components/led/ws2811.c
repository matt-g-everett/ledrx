#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "sdkconfig.h"

#include "ws2811.h"

const static char *TAG = "WS2811";

#define WS2811_RESOLUTION_HZ 10000000  // 10MHz resolution, 1 tick = 0.1us

// Target-specific RMT configuration
#if CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3: DMA mode broken with multiple channels (ESP-IDF bug #14736)
    // Using non-DMA mode with mem_block_symbols matching SOC_RMT_MEM_WORDS_PER_CHANNEL
    #define RMT_MEM_BLOCK_SYMBOLS 48
    #define RMT_USE_DMA false
    #define WS2811_ALLOC(size) malloc(size)
#elif CONFIG_IDF_TARGET_ESP32
    #define RMT_MEM_BLOCK_SYMBOLS 64
    #define RMT_USE_DMA false
    #define WS2811_ALLOC(size) malloc(size)
#else
    #error "Unsupported target - requires ESP32 or ESP32-S3"
#endif

// WS2811 timing requirements (in 0.1us ticks at 10MHz)
// WS2811 uses 400kHz protocol with 2.5us bit period
#define WS2811_T0H_TICKS    5    // 0 bit: high for 500ns
#define WS2811_T0L_TICKS    20   // 0 bit: low for 2000ns
#define WS2811_T1H_TICKS    12   // 1 bit: high for 1200ns
#define WS2811_T1L_TICKS    13   // 1 bit: low for 1300ns
#define WS2811_RESET_TICKS  500  // Reset: low for >50us

typedef struct {
    rmt_channel_handle_t tx_channel;
    rmt_encoder_handle_t encoder;
    SemaphoreHandle_t done_sem;
} ws2811_channel_t;

static ws2811_channel_t *_channels = NULL;
static uint8_t _channel_count = 0;
static rmt_sync_manager_handle_t _sync_manager = NULL;

// RMT TX encoder for WS2811
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t ws2811_bit0;
    rmt_symbol_word_t ws2811_bit1;
    rmt_symbol_word_t ws2811_reset;
    int state;  // Encoder state: 0 = encoding bytes, 1 = encoding reset
} rmt_ws2811_encoder_t;

static size_t rmt_encode_ws2811(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                const void *primary_data, size_t data_size,
                                rmt_encode_state_t *ret_state)
{
    rmt_ws2811_encoder_t *ws2811_encoder = __containerof(encoder, rmt_ws2811_encoder_t, base);
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    rmt_encoder_handle_t bytes_encoder = ws2811_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2811_encoder->copy_encoder;

    switch (ws2811_encoder->state) {
    case 0:  // Encoding RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &state);
        if (state & RMT_ENCODING_COMPLETE) {
            ws2811_encoder->state = 1;  // Move to reset code
        }
        if (state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        // fall through to send reset code
    case 1:  // Encoding reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2811_encoder->ws2811_reset,
                                                 sizeof(rmt_symbol_word_t), &state);
        if (state & RMT_ENCODING_COMPLETE) {
            ws2811_encoder->state = 0;  // Reset for next transmission
            *ret_state = RMT_ENCODING_COMPLETE;
            return encoded_symbols;
        }
        if (state & RMT_ENCODING_MEM_FULL) {
            *ret_state = RMT_ENCODING_MEM_FULL;
            return encoded_symbols;
        }
        break;
    }
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_ws2811_encoder(rmt_encoder_t *encoder)
{
    rmt_ws2811_encoder_t *ws2811_encoder = __containerof(encoder, rmt_ws2811_encoder_t, base);
    rmt_del_encoder(ws2811_encoder->bytes_encoder);
    rmt_del_encoder(ws2811_encoder->copy_encoder);
    free(ws2811_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ws2811_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ws2811_encoder_t *ws2811_encoder = __containerof(encoder, rmt_ws2811_encoder_t, base);
    rmt_encoder_reset(ws2811_encoder->bytes_encoder);
    rmt_encoder_reset(ws2811_encoder->copy_encoder);
    ws2811_encoder->state = 0;
    return ESP_OK;
}

esp_err_t rmt_new_ws2811_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_ws2811_encoder_t *ws2811_encoder = NULL;

    ws2811_encoder = calloc(1, sizeof(rmt_ws2811_encoder_t));
    if (ws2811_encoder == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ws2811_encoder->base.encode = rmt_encode_ws2811;
    ws2811_encoder->base.del = rmt_del_ws2811_encoder;
    ws2811_encoder->base.reset = rmt_ws2811_encoder_reset;

    // WS2811 bit encoding
    ws2811_encoder->ws2811_bit0.level0 = 1;
    ws2811_encoder->ws2811_bit0.duration0 = WS2811_T0H_TICKS;
    ws2811_encoder->ws2811_bit0.level1 = 0;
    ws2811_encoder->ws2811_bit0.duration1 = WS2811_T0L_TICKS;

    ws2811_encoder->ws2811_bit1.level0 = 1;
    ws2811_encoder->ws2811_bit1.duration0 = WS2811_T1H_TICKS;
    ws2811_encoder->ws2811_bit1.level1 = 0;
    ws2811_encoder->ws2811_bit1.duration1 = WS2811_T1L_TICKS;

    ws2811_encoder->ws2811_reset.level0 = 0;
    ws2811_encoder->ws2811_reset.duration0 = WS2811_RESET_TICKS;
    ws2811_encoder->ws2811_reset.level1 = 0;
    ws2811_encoder->ws2811_reset.duration1 = 0;

    // Create bytes encoder
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = ws2811_encoder->ws2811_bit0,
        .bit1 = ws2811_encoder->ws2811_bit1,
        .flags.msb_first = 1,
    };
    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2811_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        goto err;
    }

    // Create copy encoder for reset code
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2811_encoder->copy_encoder);
    if (ret != ESP_OK) {
        goto err;
    }

    *ret_encoder = &ws2811_encoder->base;
    return ESP_OK;

err:
    if (ws2811_encoder) {
        if (ws2811_encoder->bytes_encoder) {
            rmt_del_encoder(ws2811_encoder->bytes_encoder);
        }
        if (ws2811_encoder->copy_encoder) {
            rmt_del_encoder(ws2811_encoder->copy_encoder);
        }
        free(ws2811_encoder);
    }
    return ret;
}

static bool rmt_tx_done_callback(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_data)
{
    SemaphoreHandle_t done_sem = (SemaphoreHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;
    xSemaphoreGiveFromISR(done_sem, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

void ws2811_init(int *gpioNum, size_t count)
{
    esp_err_t ret;

    _channel_count = count;
    _channels = calloc(count, sizeof(ws2811_channel_t));
    if (_channels == NULL) {
        ESP_LOGE(TAG, "Failed to allocate channel memory");
        return;
    }

    ESP_LOGI(TAG, "Initializing %d WS2811 channels", _channel_count);

    for (int i = 0; i < count; i++) {
        // Create RMT TX channel
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = gpioNum[i],
            .mem_block_symbols = RMT_MEM_BLOCK_SYMBOLS,
            .resolution_hz = WS2811_RESOLUTION_HZ,
            .trans_queue_depth = 4,
            .flags.with_dma = RMT_USE_DMA,
        };
        ret = rmt_new_tx_channel(&tx_chan_config, &_channels[i].tx_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create RMT TX channel %d: %s", i, esp_err_to_name(ret));
            return;
        }

        // Create WS2811 encoder
        ret = rmt_new_ws2811_encoder(&_channels[i].encoder);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create WS2811 encoder for channel %d: %s", i, esp_err_to_name(ret));
            return;
        }

        // Create semaphore for synchronization
        _channels[i].done_sem = xSemaphoreCreateBinary();
        if (_channels[i].done_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create semaphore for channel %d", i);
            return;
        }

        // Register TX done callback
        rmt_tx_event_callbacks_t cbs = {
            .on_trans_done = rmt_tx_done_callback,
        };
        ret = rmt_tx_register_event_callbacks(_channels[i].tx_channel, &cbs, _channels[i].done_sem);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to register callbacks for channel %d: %s", i, esp_err_to_name(ret));
            return;
        }

        // Enable the channel
        ret = rmt_enable(_channels[i].tx_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable RMT channel %d: %s", i, esp_err_to_name(ret));
            return;
        }

        ESP_LOGI(TAG, "Initialized WS2811 channel %d on GPIO %d", i, gpioNum[i]);
    }

    // Create sync manager to synchronize all channels
    if (count > 1) {
        rmt_channel_handle_t *tx_channels = malloc(count * sizeof(rmt_channel_handle_t));
        if (tx_channels == NULL) {
            ESP_LOGE(TAG, "Failed to allocate sync channel array");
            return;
        }
        for (int i = 0; i < count; i++) {
            tx_channels[i] = _channels[i].tx_channel;
        }

        rmt_sync_manager_config_t sync_config = {
            .tx_channel_array = tx_channels,
            .array_size = count,
        };
        ret = rmt_new_sync_manager(&sync_config, &_sync_manager);
        free(tx_channels);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create sync manager: %s", esp_err_to_name(ret));
            // Continue without sync manager - channels will still work, just not perfectly synchronized
            _sync_manager = NULL;
        } else {
            ESP_LOGI(TAG, "Created sync manager for %d channels", count);
        }
    }
}

void ws2811_setColors(unsigned int length, RGB_t *array)
{
    if (_channels == NULL || _channel_count == 0) {
        ESP_LOGE(TAG, "WS2811 not initialized");
        return;
    }

    uint16_t leds_per_channel = length / _channel_count;
    uint16_t buffer_size = leds_per_channel * 3;  // 3 bytes (RGB) per LED

    // Prepare buffers for each channel
    uint8_t **channel_buffers = malloc(_channel_count * sizeof(uint8_t *));
    if (channel_buffers == NULL) {
        ESP_LOGE(TAG, "Failed to allocate channel buffer pointers");
        return;
    }

    for (int chan = 0; chan < _channel_count; chan++) {
        channel_buffers[chan] = WS2811_ALLOC(buffer_size);
        if (channel_buffers[chan] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate buffer for channel %d", chan);
            // Clean up already allocated buffers
            for (int j = 0; j < chan; j++) {
                free(channel_buffers[j]);
            }
            free(channel_buffers);
            return;
        }

        // Fill buffer with RGB data for this channel
        uint16_t start_led = chan * leds_per_channel;
        for (int i = 0; i < leds_per_channel; i++) {
            uint16_t led_index = start_led + i;
            channel_buffers[chan][i * 3 + 0] = array[led_index].r;
            channel_buffers[chan][i * 3 + 1] = array[led_index].g;
            channel_buffers[chan][i * 3 + 2] = array[led_index].b;
        }
    }

    // Reset sync manager before transmission to re-arm synchronization
    if (_sync_manager != NULL) {
        rmt_sync_reset(_sync_manager);
    }

    // Transmit on all channels simultaneously for synchronization
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags.eot_level = 0,
    };

    for (int chan = 0; chan < _channel_count; chan++) {
        esp_err_t ret = rmt_transmit(_channels[chan].tx_channel, _channels[chan].encoder,
                                      channel_buffers[chan], buffer_size, &tx_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit on channel %d: %s", chan, esp_err_to_name(ret));
        }
    }

    // Wait for all channels to complete transmission (ensures synchronization)
    for (int chan = 0; chan < _channel_count; chan++) {
        xSemaphoreTake(_channels[chan].done_sem, portMAX_DELAY);
    }

    // Clean up buffers
    for (int chan = 0; chan < _channel_count; chan++) {
        free(channel_buffers[chan]);
    }
    free(channel_buffers);
}
