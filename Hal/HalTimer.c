#include "HalTimer.h"
#include "stm32f0xx_conf.h"

#include <stddef.h>

#define TIMER_TICK_RATE 1000

static TimerCallback timerCallback;

void TIM16_IRQHandler(void)
{
    if (TIM_GetFlagStatus(TIM16, TIM_FLAG_Update)) {
        TIM_ClearFlag(TIM16, TIM_FLAG_Update);
    }
    if (timerCallback != NULL) {
        timerCallback();
    }
}

void halTimerInit(TimerCallback timerCb)
{
    timerCallback = timerCb;
    RCC_ClocksTypeDef rccClocks;
    RCC_GetClocksFreq(&rccClocks);
    TIM_TimeBaseInitTypeDef timInitStructure;
    TIM_TimeBaseStructInit(&timInitStructure);
    timInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timInitStructure.TIM_Period = rccClocks.PCLK_Frequency / TIMER_TICK_RATE - 1;
    timInitStructure.TIM_Prescaler = 0;
    TIM_TimeBaseInit(TIM16, &timInitStructure);
    NVIC_EnableIRQ(TIM16_IRQn);
    TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM16, ENABLE);
}
