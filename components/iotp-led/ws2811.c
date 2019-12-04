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

#define TOTAL_BLOCKS 4
#define PULSES_PER_BLOCK 64

#define MAX_CHANNELS 4

const static char *TAG = "WS2811";

static uint8_t *ws2811_buffer[MAX_CHANNELS];
static uint16_t ws2811_pos[MAX_CHANNELS], ws2811_len,
    ws2811_half[MAX_CHANNELS], ws2811_bufIsDirty[MAX_CHANNELS];
static xSemaphoreHandle ws2811_sem[MAX_CHANNELS];
static intr_handle_t rmt_intr_handle;
static uint32_t _blocks_per_channel;
static uint32_t _channel_pulses;
static uint32_t _write_pulses;
static uint8_t _channel_count;

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

    offset = ws2811_half[rmtChannel] * _write_pulses;
    ws2811_half[rmtChannel] = !ws2811_half[rmtChannel];

    len = ws2811_len - ws2811_pos[rmtChannel];
    if (len > (_write_pulses / 8))
        len = (_write_pulses / 8);

    if (!len)
    {
        if (!ws2811_bufIsDirty[rmtChannel])
        {
            return;
        }

        for (i = 0; i < _write_pulses; i++)
            RMTMEM.chan[rmtChannel].data32[i + offset].val = 0;

        ws2811_bufIsDirty[rmtChannel] = 0;
        return;
    }
    ws2811_bufIsDirty[rmtChannel] = 1;

    for (i = 0; i < len; i++)
    {
        bit = ws2811_buffer[rmtChannel][i + ws2811_pos[rmtChannel]];
        for (j = 0; j < 8; j++, bit <<= 1)
        {
            RMTMEM.chan[rmtChannel].data32[j + i * 8 + offset].val = ((bit >> 7) & 0x01) ? high.val : low.val;
        }

        if (i + ws2811_pos[rmtChannel] == ws2811_len - 1)
        {
            RMTMEM.chan[rmtChannel].data32[7 + i * 8 + offset].duration1 = RESET_TICKS;
        }
    }

    for (i *= 8; i < _write_pulses; i++)
        RMTMEM.chan[rmtChannel].data32[i + offset].val = 0;

    ws2811_pos[rmtChannel] += len;
    return;
}

void ws2811_handleInterrupt(void *arg)
{
    portBASE_TYPE taskAwoken = 0;

    if (RMT.int_st.ch0_tx_thr_event)
    {
        ws2811_copy(0);
        RMT.int_clr.ch0_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch0_tx_end && ws2811_sem[0])
    {
        xSemaphoreGiveFromISR(ws2811_sem[0], &taskAwoken);
        RMT.int_clr.ch0_tx_end = 1;
    }
    else if (RMT.int_st.ch1_tx_thr_event)
    {
        ws2811_copy(1);
        RMT.int_clr.ch1_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch1_tx_end && ws2811_sem[1])
    {
        xSemaphoreGiveFromISR(ws2811_sem[1], &taskAwoken);
        RMT.int_clr.ch1_tx_end = 1;
    }
    else if (RMT.int_st.ch2_tx_thr_event)
    {
        ws2811_copy(2);
        RMT.int_clr.ch2_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch2_tx_end && ws2811_sem[2])
    {
        xSemaphoreGiveFromISR(ws2811_sem[2], &taskAwoken);
        RMT.int_clr.ch2_tx_end = 1;
    }
    else if (RMT.int_st.ch3_tx_thr_event)
    {
        ws2811_copy(3);
        RMT.int_clr.ch3_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch3_tx_end && ws2811_sem[3])
    {
        xSemaphoreGiveFromISR(ws2811_sem[3], &taskAwoken);
        RMT.int_clr.ch3_tx_end = 1;
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

    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);
    RMT.apb_conf.fifo_mask = 1; //enable memory access, instead of FIFO mode.
    RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer

    for (chan = 0; chan < _channel_count; chan++) {
        rmt_set_pin((rmt_channel_t)chan, RMT_MODE_TX, (gpio_num_t)gpioNum[chan]);
        ws2811_initRMTChannel(chan);
        RMT.tx_lim_ch[chan].limit = _write_pulses;
    }

    if (_channel_count > 0) {
        RMT.int_ena.ch0_tx_thr_event = 1;
        RMT.int_ena.ch0_tx_end = 1;
    }
    else if (_channel_count > 1) {
        RMT.int_ena.ch1_tx_thr_event = 1;
        RMT.int_ena.ch1_tx_end = 1;
    }
    else if (_channel_count > 3) {
        RMT.int_ena.ch2_tx_thr_event = 1;
        RMT.int_ena.ch2_tx_end = 1;
        RMT.int_ena.ch3_tx_thr_event = 1;
        RMT.int_ena.ch3_tx_end = 1;
    }

    esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, ws2811_handleInterrupt, NULL, &rmt_intr_handle);

    return;
}

void ws2811_setColors(unsigned int length, RGB_t *array)
{
    uint8_t chan;
    uint16_t i, j, buffer_end;

    uint16_t full_len = (length * 3) * sizeof(uint8_t);
    ws2811_len = full_len / _channel_count;
    uint16_t array_len_per_channel = length / _channel_count;

    j = 0;
    for (chan = 0; chan < _channel_count; chan++) {
        ws2811_buffer[chan] = malloc(ws2811_len);
        buffer_end = (chan + 1) * array_len_per_channel;
        for (i = 0; j < buffer_end; j++, i++)
        {
            ws2811_buffer[chan][0 + i * 3] = array[j].r;
            ws2811_buffer[chan][1 + i * 3] = array[j].g;
            ws2811_buffer[chan][2 + i * 3] = array[j].b;
        }

        ws2811_pos[chan] = 0;
        ws2811_half[chan] = 0;
    }

    for (chan = 0; chan < _channel_count; chan++) {

        ws2811_copy(chan);

        if (ws2811_pos[chan] < ws2811_len)
            ws2811_copy(chan);

        ws2811_sem[chan] = xSemaphoreCreateBinary();

        RMT.conf_ch[chan].conf1.mem_rd_rst = 1;
        RMT.conf_ch[chan].conf1.tx_start = 1;
    }

    for (chan = 0; chan < _channel_count; chan++) {

        xSemaphoreTake(ws2811_sem[chan], portMAX_DELAY);
        vSemaphoreDelete(ws2811_sem[chan]);
        ws2811_sem[chan] = NULL;

        free(ws2811_buffer[chan]);
    }

    return;
}
