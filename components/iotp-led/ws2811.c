/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Uses the RMT peripheral on the ESP32 for very accurate timing of
 * signals sent to the WS2811 LEDs.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "xtensa/core-macros.h"
#include <soc/rmt_struct.h>
#include <soc/dport_reg.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include "esp_log.h"
#include <esp_intr_alloc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <driver/rmt.h>

#include "esp_log.h"

#include "ws2811.h"

#define ETS_RMT_CTRL_INUM 18
#define ESP_RMT_CTRL_DISABLE ESP_RMT_CTRL_DIABLE /* Typo in esp_intr.h */

#define DIVIDER 4     /* Above 4, timings start to deviate*/
#define DURATION 12.5 /* minimum time of a single RMT duration \
        in nanoseconds based on clock */

#define ONE_HIGH_TICKS 24
#define ONE_LOW_TICKS 26
#define ZERO_HIGH_TICKS 10
#define ZERO_LOW_TICKS 40
#define RESET_TICKS 1000

#define TOTAL_BLOCKS 8
#define PULSES_PER_BLOCK 64

#define MAX_CHANNELS 8

const static char *TAG = "WS2811";

typedef struct {
    uint8_t *buffer;
    uint16_t buffer_len;
    uint8_t dirty;
    uint16_t pos;
    uint8_t half;
    xSemaphoreHandle sem;
} ws2811_channel_send_state_t;

static intr_handle_t rmt_intr_handle;

static ws2811_channel_send_state_t _send_states[MAX_CHANNELS];
static uint32_t _blocks_per_channel;
static uint32_t _channel_pulses;
static uint32_t _write_pulses;
static uint8_t _channel_count;
static uint8_t _channel_to_rmt[MAX_CHANNELS];

static rmt_item32_t high = {
    .duration0 = ONE_HIGH_TICKS, .level0 = 1, .duration1 = ONE_LOW_TICKS, .level1 = 0};
static rmt_item32_t low = {
    .duration0 = ZERO_HIGH_TICKS, .level0 = 1, .duration1 = ZERO_LOW_TICKS, .level1 = 0};

void ws2811_initRMTChannel(uint8_t rmtChannel)
{
    RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
    RMT.conf_ch[rmtChannel].conf0.mem_size = _blocks_per_channel;
    RMT.conf_ch[rmtChannel].conf0.carrier_en = 0;
    RMT.conf_ch[rmtChannel].conf0.carrier_out_lv = 1;
    RMT.conf_ch[rmtChannel].conf0.mem_pd = 0;

    RMT.conf_ch[rmtChannel].conf1.rx_en = 0;
    RMT.conf_ch[rmtChannel].conf1.mem_owner = 0;
    RMT.conf_ch[rmtChannel].conf1.tx_conti_mode = 0; //loop back mode.
    RMT.conf_ch[rmtChannel].conf1.ref_always_on = 1; // use apb clock: 80M
    RMT.conf_ch[rmtChannel].conf1.idle_out_en = 1;
    RMT.conf_ch[rmtChannel].conf1.idle_out_lv = 0;

    return;
}

void ws2811_copy(uint8_t rmtChannel)
{
    uint16_t i, offset, len;
    uint8_t j, bit;

    ws2811_channel_send_state_t *send_state = _send_states + rmtChannel;

    offset = send_state->half * _write_pulses;
    send_state->half = !send_state->half;

    len = send_state->buffer_len - send_state->pos;
    if (len > (_write_pulses / 8))
        len = (_write_pulses / 8);

    if (!len)
    {
        if (!send_state->dirty)
        {
            return;
        }

        for (i = 0; i < _write_pulses; i++)
            RMTMEM.chan[rmtChannel].data32[i + offset].val = 0;

        send_state->dirty = 0;
        return;
    }
    send_state->dirty = 1;

    for (i = 0; i < len; i++)
    {
        bit = send_state->buffer[i + send_state->pos];
        for (j = 0; j < 8; j++, bit <<= 1)
        {
            RMTMEM.chan[rmtChannel].data32[j + i * 8 + offset].val = ((bit >> 7) & 0x01) ? high.val : low.val;
        }

        if (i + send_state->pos == send_state->buffer_len - 1)
        {
            RMTMEM.chan[rmtChannel].data32[7 + i * 8 + offset].duration1 = RESET_TICKS;
        }
    }

    for (i *= 8; i < _write_pulses; i++)
        RMTMEM.chan[rmtChannel].data32[i + offset].val = 0;

    send_state->pos += len;
    return;
}

void ws2811_handleInterrupt(void *arg)
{
    portBASE_TYPE taskAwoken = 0;

    // Handle channel 0
    if (RMT.int_st.ch0_tx_thr_event)
    {
        ws2811_copy(0);
        RMT.int_clr.ch0_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch0_tx_end && _send_states[0].sem)
    {
        xSemaphoreGiveFromISR(_send_states[0].sem, &taskAwoken);
        RMT.int_clr.ch0_tx_end = 1;
    }
    // Handle channel 1
    else if (RMT.int_st.ch1_tx_thr_event)
    {
        ws2811_copy(1);
        RMT.int_clr.ch1_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch1_tx_end && _send_states[1].sem)
    {
        xSemaphoreGiveFromISR(_send_states[1].sem, &taskAwoken);
        RMT.int_clr.ch1_tx_end = 1;
    }
    // Handle channel 2
    else if (RMT.int_st.ch2_tx_thr_event)
    {
        ws2811_copy(2);
        RMT.int_clr.ch2_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch2_tx_end && _send_states[2].sem)
    {
        xSemaphoreGiveFromISR(_send_states[2].sem, &taskAwoken);
        RMT.int_clr.ch2_tx_end = 1;
    }
    // Handle channel 3
    else if (RMT.int_st.ch3_tx_thr_event)
    {
        ws2811_copy(3);
        RMT.int_clr.ch3_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch3_tx_end && _send_states[3].sem)
    {
        xSemaphoreGiveFromISR(_send_states[3].sem, &taskAwoken);
        RMT.int_clr.ch3_tx_end = 1;
    }
    // Handle channel 4
    if (RMT.int_st.ch4_tx_thr_event)
    {
        ws2811_copy(4);
        RMT.int_clr.ch4_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch4_tx_end && _send_states[4].sem)
    {
        xSemaphoreGiveFromISR(_send_states[4].sem, &taskAwoken);
        RMT.int_clr.ch4_tx_end = 1;
    }
    // Handle channel 5
    else if (RMT.int_st.ch5_tx_thr_event)
    {
        ws2811_copy(5);
        RMT.int_clr.ch5_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch5_tx_end && _send_states[5].sem)
    {
        xSemaphoreGiveFromISR(_send_states[5].sem, &taskAwoken);
        RMT.int_clr.ch5_tx_end = 1;
    }
    // Handle channel 6
    else if (RMT.int_st.ch6_tx_thr_event)
    {
        ws2811_copy(6);
        RMT.int_clr.ch6_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch6_tx_end && _send_states[6].sem)
    {
        xSemaphoreGiveFromISR(_send_states[6].sem, &taskAwoken);
        RMT.int_clr.ch6_tx_end = 1;
    }
    // Handle channel 7
    else if (RMT.int_st.ch7_tx_thr_event)
    {
        ws2811_copy(7);
        RMT.int_clr.ch7_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch7_tx_end && _send_states[7].sem)
    {
        xSemaphoreGiveFromISR(_send_states[7].sem, &taskAwoken);
        RMT.int_clr.ch7_tx_end = 1;
    }

    return;
}

void ws2811_init(int *gpioNum, size_t count)
{
    uint8_t chan;

    _blocks_per_channel = TOTAL_BLOCKS / count;
    _channel_pulses = _blocks_per_channel * PULSES_PER_BLOCK;
    _write_pulses = _channel_pulses / 2; // We write half of the buffer at a time
    _channel_count = count;

    ESP_LOGI(TAG, "Initialising bpc %d, cp %d, wp %d, cc %d", _blocks_per_channel, _channel_pulses, _write_pulses, _channel_count);

    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);
    RMT.apb_conf.fifo_mask = 1; //enable memory access, instead of FIFO mode.
    RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer

    RMT.int_ena.ch0_tx_thr_event = 1;
    RMT.int_ena.ch0_tx_end = 1;
    _channel_to_rmt[0] = 0;

    if (_channel_count == 2) {
        RMT.int_ena.ch4_tx_thr_event = 1;
        RMT.int_ena.ch4_tx_end = 1;
        _channel_to_rmt[1] = 4;
    }
    else if (_channel_count == 4) {
        RMT.int_ena.ch2_tx_thr_event = 1;
        RMT.int_ena.ch2_tx_end = 1;
        _channel_to_rmt[1] = 2;

        RMT.int_ena.ch4_tx_thr_event = 1;
        RMT.int_ena.ch4_tx_end = 1;
        _channel_to_rmt[2] = 4;

        RMT.int_ena.ch6_tx_thr_event = 1;
        RMT.int_ena.ch6_tx_end = 1;
        _channel_to_rmt[3] = 6;
    }
    else if (_channel_count == 8) {
        RMT.int_ena.ch1_tx_thr_event = 1;
        RMT.int_ena.ch1_tx_end = 1;
        _channel_to_rmt[1] = 1;

        RMT.int_ena.ch2_tx_thr_event = 1;
        RMT.int_ena.ch2_tx_end = 1;
        _channel_to_rmt[2] = 2;

        RMT.int_ena.ch3_tx_thr_event = 1;
        RMT.int_ena.ch3_tx_end = 1;
        _channel_to_rmt[3] = 3;

        RMT.int_ena.ch4_tx_thr_event = 1;
        RMT.int_ena.ch4_tx_end = 1;
        _channel_to_rmt[4] = 4;

        RMT.int_ena.ch5_tx_thr_event = 1;
        RMT.int_ena.ch5_tx_end = 1;
        _channel_to_rmt[5] = 5;

        RMT.int_ena.ch6_tx_thr_event = 1;
        RMT.int_ena.ch6_tx_end = 1;
        _channel_to_rmt[6] = 6;

        RMT.int_ena.ch7_tx_thr_event = 1;
        RMT.int_ena.ch7_tx_end = 1;
        _channel_to_rmt[7] = 7;
    }

    for (chan = 0; chan < _channel_count; chan++) {
        uint8_t rmt_channel = _channel_to_rmt[chan];
        rmt_set_pin((rmt_channel_t)rmt_channel, RMT_MODE_TX, (gpio_num_t)gpioNum[chan]);
        ws2811_initRMTChannel(rmt_channel);
        RMT.tx_lim_ch[rmt_channel].limit = _write_pulses;

        ESP_LOGI(TAG, "Initialised RMT channel %d on gpio %d", rmt_channel, gpioNum[chan]);
    }

    esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, ws2811_handleInterrupt, NULL, &rmt_intr_handle);

    return;
}

void ws2811_setColors(unsigned int length, RGB_t *array)
{
    uint8_t chan;
    uint16_t i, j, buffer_end;

    uint16_t full_len = (length * 3) * sizeof(uint8_t);
    uint16_t channel_buffer_len = full_len / _channel_count;
    uint16_t array_len_per_channel = length / _channel_count;

    j = 0;
    for (chan = 0; chan < _channel_count; chan++) {
        uint8_t rmt_channel = _channel_to_rmt[chan];
        ws2811_channel_send_state_t *send_state = _send_states + rmt_channel;

        send_state->buffer = malloc(channel_buffer_len);
        send_state->buffer_len = channel_buffer_len;
        buffer_end = (chan + 1) * array_len_per_channel;
        for (i = 0; j < buffer_end; j++, i++)
        {
            send_state->buffer[0 + i * 3] = array[j].r;
            send_state->buffer[1 + i * 3] = array[j].g;
            send_state->buffer[2 + i * 3] = array[j].b;
        }

        send_state->pos = 0;
        send_state->half = 0;
    }

    for (chan = 0; chan < _channel_count; chan++) {
        uint8_t rmt_channel = _channel_to_rmt[chan];
        ws2811_channel_send_state_t *send_state = _send_states + rmt_channel;

        ws2811_copy(rmt_channel);
        if (send_state->pos < send_state->buffer_len)
            ws2811_copy(rmt_channel);

        send_state->sem = xSemaphoreCreateBinary();

        RMT.conf_ch[rmt_channel].conf1.mem_rd_rst = 1;
        RMT.conf_ch[rmt_channel].conf1.tx_start = 1;
    }

    for (chan = 0; chan < _channel_count; chan++) {
        uint8_t rmt_channel = _channel_to_rmt[chan];
        ws2811_channel_send_state_t *send_state = _send_states + rmt_channel;

        xSemaphoreTake(send_state->sem, portMAX_DELAY);
        vSemaphoreDelete(send_state->sem);
        send_state->sem = NULL;

        free(send_state->buffer);
    }

    return;
}
