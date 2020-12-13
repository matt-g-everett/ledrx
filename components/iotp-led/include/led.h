#include "freertos/FreeRTOS.h"

#include "pixels.h"

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*led_ack)(uint8_t ackID);
typedef void (*led_log)(char *message);

void led_initialise(led_log log_callback, led_ack ack_callback, int *gpios, size_t count);
void led_set_running(uint8_t running);
uint8_t led_push_stream(char *data);
void led_task(void *pParam);

#ifdef __cplusplus
}
#endif

#endif
