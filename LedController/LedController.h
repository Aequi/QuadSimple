#ifndef __LED_CONTROLLER_H__
#define __LED_CONTROLLER_H__

#include <stdint.h>

typedef enum LedControllerChannel {
    LED_CONTROLLER_CHANNEL_GREEN_LEFT,
    LED_CONTROLLER_CHANNEL_RED_LEFT,
    LED_CONTROLLER_CHANNEL_RED_RIGHT,
    LED_CONTROLLER_CHANNEL_GREEN_RIGHT,
} LedControllerChannel;

void ledControllerSetLedLevel(LedControllerChannel ledChannel, uint32_t brightnessLevel);
void ledControllerEnableLeds(void);

#endif
