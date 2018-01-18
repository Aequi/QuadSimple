#include "TimersHal.h"
#include "LedController.h"

void ledControllerSetLedLevel(LedControllerChannel ledChannel, uint32_t brightnessLevel)
{
    timersHalSetPwmChannelVal((TimersHalPwmChannel) ((uint32_t) ledChannel + (uint32_t) TIMERS_HAL_PWM_CHANNEL_4), brightnessLevel);
}

void ledControllerEnableLeds(void)
{
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_4, true);
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_5, true);
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_6, true);
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_7, true);
}
