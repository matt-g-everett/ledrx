#include "freertos/FreeRTOS.h"

#include "pixels.h"

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

void led_initialise(int gpio_num);
void led_set_running(uint8_t running);
uint8_t led_push_stream(char *data);
void led_task(void *pParam);

#ifdef __cplusplus
}
#endif

#endif
