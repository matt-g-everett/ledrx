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

#define MAX_PULSES 128
#define BLOCK_PULSES (MAX_PULSES * 2)

#define RMTCHANNEL 0

static uint8_t *ws2811_buffer = NULL;
static uint16_t ws2811_pos, ws2811_len, ws2811_half, ws2812_bufIsDirty;
static xSemaphoreHandle ws2811_sem = NULL;
static intr_handle_t rmt_intr_handle = NULL;

static rmt_item32_t high = {
    .duration0 = ONE_HIGH_TICKS, .level0 = 1, .duration1 = ONE_LOW_TICKS, .level1 = 0};
static rmt_item32_t low = {
    .duration0 = ZERO_HIGH_TICKS, .level0 = 1, .duration1 = ZERO_LOW_TICKS, .level1 = 0};

void ws2811_initRMTChannel(int rmtChannel)
{
    RMT.apb_conf.fifo_mask = 1; //enable memory access, instead of FIFO mode.
    RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer
    RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
    RMT.conf_ch[rmtChannel].conf0.mem_size = 4;
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

void ws2811_copy()
{
    unsigned int i, j, offset, len, bit;

    offset = ws2811_half * MAX_PULSES;
    ws2811_half = !ws2811_half;

    len = ws2811_len - ws2811_pos;
    if (len > (MAX_PULSES / 8))
        len = (MAX_PULSES / 8);

    if (!len)
    {
        if (!ws2812_bufIsDirty)
        {
            return;
        }

        for (i = 0; i < MAX_PULSES; i++)
            RMTMEM.chan[RMTCHANNEL].data32[i + offset].val = 0;

        ws2812_bufIsDirty = 0;
        return;
    }
    ws2812_bufIsDirty = 1;

    for (i = 0; i < len; i++)
    {
        bit = ws2811_buffer[i + ws2811_pos];
        for (j = 0; j < 8; j++, bit <<= 1)
        {
            RMTMEM.chan[RMTCHANNEL].data32[j + i * 8 + offset].val = ((bit >> 7) & 0x01) ? high.val : low.val;
        }

        if (i + ws2811_pos == ws2811_len - 1)
        {
            RMTMEM.chan[RMTCHANNEL].data32[7 + i * 8 + offset].duration1 = RESET_TICKS;
        }
    }

    for (i *= 8; i < MAX_PULSES; i++)
        RMTMEM.chan[RMTCHANNEL].data32[i + offset].val = 0;

    ws2811_pos += len;
    return;
}

void ws2811_handleInterrupt(void *arg)
{
    portBASE_TYPE taskAwoken = 0;

    if (RMT.int_st.ch0_tx_thr_event)
    {
        ws2811_copy();
        RMT.int_clr.ch0_tx_thr_event = 1;
    }
    else if (RMT.int_st.ch0_tx_end && ws2811_sem)
    {
        xSemaphoreGiveFromISR(ws2811_sem, &taskAwoken);
        RMT.int_clr.ch0_tx_end = 1;
    }

    return;
}

void ws2811_init(int gpioNum)
{
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);

    rmt_set_pin((rmt_channel_t)RMTCHANNEL, RMT_MODE_TX, (gpio_num_t)gpioNum);

    ws2811_initRMTChannel(RMTCHANNEL);

    RMT.tx_lim_ch[RMTCHANNEL].limit = MAX_PULSES;
    RMT.int_ena.ch0_tx_thr_event = 1;
    RMT.int_ena.ch0_tx_end = 1;

    esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, ws2811_handleInterrupt, NULL, &rmt_intr_handle);

    return;
}

void ws2811_setColors(unsigned int length, RGB_t *array)
{
    unsigned int i;

    ws2811_len = (length * 3) * sizeof(uint8_t);
    ws2811_buffer = malloc(ws2811_len);

    for (i = 0; i < length; i++)
    {
        ws2811_buffer[0 + i * 3] = array[i].r;
        ws2811_buffer[1 + i * 3] = array[i].g;
        ws2811_buffer[2 + i * 3] = array[i].b;
    }

    ws2811_pos = 0;
    ws2811_half = 0;

    ws2811_copy();

    if (ws2811_pos < ws2811_len)
        ws2811_copy();

    ws2811_sem = xSemaphoreCreateBinary();

    //ets_printf("st\n");
    RMT.conf_ch[RMTCHANNEL].conf1.mem_rd_rst = 1;
    RMT.conf_ch[RMTCHANNEL].conf1.tx_start = 1;

    xSemaphoreTake(ws2811_sem, portMAX_DELAY);
    vSemaphoreDelete(ws2811_sem);
    ws2811_sem = NULL;

    free(ws2811_buffer);

    return;
}
