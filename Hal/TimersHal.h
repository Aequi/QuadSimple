#ifndef __TIMERS_HAL_H__
#define __TIMERS_HAL_H__

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx.h"

#define TIMER_HAL_PWM_MAX       1023
#define TIMER_HAL_AUX_TICK_RATE 10000
#define TIMER_HAL_AUX_CB_RATE   25

typedef enum TimersHalPwmChannel {
    TIMERS_HAL_PWM_CHANNEL_0,
    TIMERS_HAL_PWM_CHANNEL_1,
    TIMERS_HAL_PWM_CHANNEL_2,
    TIMERS_HAL_PWM_CHANNEL_3,
    TIMERS_HAL_PWM_CHANNEL_4,
    TIMERS_HAL_PWM_CHANNEL_5,
    TIMERS_HAL_PWM_CHANNEL_6,
    TIMERS_HAL_PWM_CHANNEL_7,
} TimersHalPwmChannel;

void timersHalInit();

void timersHalEnablePwmChannel(TimersHalPwmChannel pwmChannel, bool isEnabled);
void timersHalSetPwmChannelVal(TimersHalPwmChannel pwmChannel, uint32_t value);
uint32_t timersHalGetAuxCounterValue(void);
void delayMs(uint32_t delayTime);

static inline void timersHalSetPwmFastChannel0(uint32_t value)
{
    TIM3->CCR2 = value;
}

static inline void timersHalSetPwmFastChannel1(uint32_t value)
{
    TIM3->CCR4 = value;
}

static inline void timersHalSetPwmFastChannel2(uint32_t value)
{
    TIM4->CCR2 = value;
}

static inline void timersHalSetPwmFastChannel3(uint32_t value)
{
    TIM4->CCR4 = value;
}

#endif
