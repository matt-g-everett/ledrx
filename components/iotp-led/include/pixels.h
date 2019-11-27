#include "freertos/FreeRTOS.h"

#ifndef __PIXELS_H
#define __PIXELS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    union {
        struct {
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };
        uint8_t subpixels[3];
    };
} RGB_t;

typedef struct frame_t {
    uint16_t len;
    RGB_t data[CONFIG_LED_NUM_PIXELS];
} FRAME_t;

#ifdef __cplusplus
}
#endif

#endif
