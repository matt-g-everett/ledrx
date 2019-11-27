/* Created 19 Nov 2016 by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * This is a driver for the WS2812 RGB LEDs using the RMT peripheral on the ESP32.
 *
 * This code is placed in the public domain (or CC0 licensed, at your option).
 */

#include <stdint.h>
#include "pixels.h"

#ifndef WS2811_DRIVER_H
#define WS2811_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

extern void ws2811_init(int gpioNum);
extern void ws2811_setColors(unsigned int length, RGB_t *array);

#ifdef __cplusplus
}
#endif

#endif /* WS2811_DRIVER_H */
