#include "TimersHal.h"
#include "HalCommon.h"
#include "stm32f4xx_conf.h"
#include <stddef.h>

#define TIMERS_HAL_LED_RED_LEFT_PWM_PIN             GPIO_Pin_0
#define TIMERS_HAL_LED_RED_LEFT_PWM_PIN_SRC         GPIO_PinSource0
#define TIMERS_HAL_LED_RED_LEFT_PWM_PORT            GPIOB

#define TIMERS_HAL_BACK_LEFT_MOTOR_PWM_PIN          GPIO_Pin_1
#define TIMERS_HAL_BACK_LEFT_MOTOR_PWM_PIN_SRC      GPIO_PinSource1
#define TIMERS_HAL_BACK_LEFT_MOTOR_PWM_PORT         GPIOB

#define TIMERS_HAL_LED_RED_RIGHT_PWM_PIN            GPIO_Pin_6
#define TIMERS_HAL_LED_RED_RIGHT_PWM_PIN_SRC        GPIO_PinSource6
#define TIMERS_HAL_LED_RED_RIGHT_PWM_PORT           GPIOB

#define TIMERS_HAL_BACK_RIGHT_MOTOR_PWM_PIN         GPIO_Pin_7
#define TIMERS_HAL_BACK_RIGHT_MOTOR_PWM_PIN_SRC     GPIO_PinSource7
#define TIMERS_HAL_BACK_RIGHT_MOTOR_PWM_PORT        GPIOB

#define TIMERS_HAL_LED_GREEN_RIGHT_PWM_PIN          GPIO_Pin_8
#define TIMERS_HAL_LED_GREEN_RIGHT_PWM_PIN_SRC      GPIO_PinSource8
#define TIMERS_HAL_LED_GREEN_RIGHT_PWM_PORT         GPIOB

#define TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PIN        GPIO_Pin_9
#define TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PIN_SRC    GPIO_PinSource9
#define TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PORT       GPIOB

#define TIMERS_HAL_LED_GREEN_LEFT_PWM_PIN           GPIO_Pin_6
#define TIMERS_HAL_LED_GREEN_LEFT_PWM_PIN_SRC       GPIO_PinSource6
#define TIMERS_HAL_LED_GREEN_LEFT_PWM_PORT          GPIOA

#define TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PIN         GPIO_Pin_7
#define TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PIN_SRC     GPIO_PinSource7
#define TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PORT        GPIOA

#define TIMERS_HAL_PWM_HW_UNIT_0_PINS_AF            GPIO_AF_TIM3
#define TIMERS_HAL_PWM_HW_UNIT_1_PINS_AF            GPIO_AF_TIM4

#define TIMERS_HAL_PWM_HW_UNIT_0                    TIM3
#define TIMERS_HAL_PWM_HW_UNIT_1                    TIM4
#define TIMERS_HAL_AUX_HW_UNIT                      TIM10
#define TIMERS_HAL_AUX_IRQ                          TIM1_UP_TIM10_IRQHandler
#define TIMERS_HAL_AUX_IRQn                         TIM1_UP_TIM10_IRQn

static void (*timerCallback)(void);

void timersHalInit(void (*timerCb)(void))
{
    timerCallback = timerCb;
    GPIO_InitTypeDef gpioInitStructure = {.GPIO_Pin = TIMERS_HAL_LED_RED_LEFT_PWM_PIN | TIMERS_HAL_BACK_LEFT_MOTOR_PWM_PIN |
                                                      TIMERS_HAL_LED_RED_RIGHT_PWM_PIN | TIMERS_HAL_BACK_RIGHT_MOTOR_PWM_PIN |
                                                      TIMERS_HAL_LED_GREEN_RIGHT_PWM_PIN | TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PIN,

                                          .GPIO_Mode = GPIO_Mode_AF,
                                          .GPIO_Speed = GPIO_Speed_50MHz,
                                          .GPIO_OType = GPIO_OType_PP,
                                          .GPIO_PuPd = GPIO_PuPd_UP};
    GPIO_Init(TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PORT, &gpioInitStructure);
    GPIO_PinAFConfig(TIMERS_HAL_LED_RED_LEFT_PWM_PORT, TIMERS_HAL_LED_RED_LEFT_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_0_PINS_AF);
    GPIO_PinAFConfig(TIMERS_HAL_BACK_LEFT_MOTOR_PWM_PORT, TIMERS_HAL_BACK_LEFT_MOTOR_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_0_PINS_AF);
    GPIO_PinAFConfig(TIMERS_HAL_LED_RED_RIGHT_PWM_PORT, TIMERS_HAL_LED_RED_RIGHT_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_1_PINS_AF);
    GPIO_PinAFConfig(TIMERS_HAL_BACK_RIGHT_MOTOR_PWM_PORT, TIMERS_HAL_BACK_RIGHT_MOTOR_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_1_PINS_AF);
    GPIO_PinAFConfig(TIMERS_HAL_LED_GREEN_RIGHT_PWM_PORT, TIMERS_HAL_LED_GREEN_RIGHT_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_1_PINS_AF);
    GPIO_PinAFConfig(TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PORT, TIMERS_HAL_FRONT_RIGHT_MOTOR_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_1_PINS_AF);

    gpioInitStructure.GPIO_Pin = TIMERS_HAL_LED_GREEN_LEFT_PWM_PIN | TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PIN;
    GPIO_Init(TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PORT, &gpioInitStructure);
    GPIO_PinAFConfig(TIMERS_HAL_LED_GREEN_LEFT_PWM_PORT, TIMERS_HAL_LED_GREEN_LEFT_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_0_PINS_AF);
    GPIO_PinAFConfig(TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PORT, TIMERS_HAL_FRONT_LEFT_MOTOR_PWM_PIN_SRC, TIMERS_HAL_PWM_HW_UNIT_0_PINS_AF);

    TIM_TimeBaseInitTypeDef timBaseInitStructure = {.TIM_ClockDivision = TIM_CKD_DIV1,
                                                    .TIM_CounterMode = TIM_CounterMode_Up,
                                                    .TIM_Prescaler = 0,
                                                    .TIM_Period = TIMER_HAL_PWM_MAX,
                                                    .TIM_RepetitionCounter = 0x0000};

    TIM_TimeBaseInit(TIMERS_HAL_PWM_HW_UNIT_0, &timBaseInitStructure);
    TIM_ARRPreloadConfig(TIMERS_HAL_PWM_HW_UNIT_0, ENABLE);

    TIM_TimeBaseInit(TIMERS_HAL_PWM_HW_UNIT_1, &timBaseInitStructure);
    TIM_ARRPreloadConfig(TIMERS_HAL_PWM_HW_UNIT_1, ENABLE);

    RCC_ClocksTypeDef rccClocks;
    RCC_GetClocksFreq(&rccClocks);

    timBaseInitStructure.TIM_Prescaler = rccClocks.SYSCLK_Frequency / TIMER_HAL_AUX_TICK_RATE - 1;
    timBaseInitStructure.TIM_Period = TIMER_HAL_AUX_TICK_RATE / TIMER_HAL_AUX_CB_RATE - 1;
    TIM_TimeBaseInit(TIMERS_HAL_AUX_HW_UNIT, &timBaseInitStructure);

    TIM_OCInitTypeDef timOcInitStructure;
    TIM_OCStructInit(&timOcInitStructure);
    timOcInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    timOcInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    timOcInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    timOcInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    timOcInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIMERS_HAL_PWM_HW_UNIT_0, &timOcInitStructure);
    TIM_OC1PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCPreload_Enable);
    TIM_OC1FastConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCFast_Disable);

    TIM_OC2Init(TIMERS_HAL_PWM_HW_UNIT_0, &timOcInitStructure);
    TIM_OC2PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCPreload_Enable);
    TIM_OC2FastConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCFast_Disable);

    TIM_OC3Init(TIMERS_HAL_PWM_HW_UNIT_0, &timOcInitStructure);
    TIM_OC3PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCPreload_Enable);
    TIM_OC3FastConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCFast_Disable);

    TIM_OC4Init(TIMERS_HAL_PWM_HW_UNIT_0, &timOcInitStructure);
    TIM_OC4PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCPreload_Enable);
    TIM_OC4FastConfig(TIMERS_HAL_PWM_HW_UNIT_0, TIM_OCFast_Disable);

    TIM_OC1Init(TIMERS_HAL_PWM_HW_UNIT_1, &timOcInitStructure);
    TIM_OC1PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCPreload_Enable);
    TIM_OC1FastConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCFast_Disable);

    TIM_OC2Init(TIMERS_HAL_PWM_HW_UNIT_1, &timOcInitStructure);
    TIM_OC2PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCPreload_Enable);
    TIM_OC2FastConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCFast_Disable);

    TIM_OC3Init(TIMERS_HAL_PWM_HW_UNIT_1, &timOcInitStructure);
    TIM_OC3PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCPreload_Enable);
    TIM_OC3FastConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCFast_Disable);

    TIM_OC4Init(TIMERS_HAL_PWM_HW_UNIT_1, &timOcInitStructure);
    TIM_OC4PreloadConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCPreload_Enable);
    TIM_OC4FastConfig(TIMERS_HAL_PWM_HW_UNIT_1, TIM_OCFast_Disable);

    TIM_ITConfig(TIMERS_HAL_AUX_HW_UNIT, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIMERS_HAL_AUX_IRQn);

    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_0, false);
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_1, false);
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_2, false);
    timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_3, false);

    TIM_Cmd(TIMERS_HAL_PWM_HW_UNIT_0, ENABLE);
    TIM_Cmd(TIMERS_HAL_PWM_HW_UNIT_1, ENABLE);
    TIM_Cmd(TIMERS_HAL_AUX_HW_UNIT, ENABLE);
}

static const struct timerChannels {
    uint16_t channel;
    TIM_TypeDef *timer;
} timChannels[] = {
    {TIM_Channel_2, TIMERS_HAL_PWM_HW_UNIT_0}, {TIM_Channel_4, TIMERS_HAL_PWM_HW_UNIT_0},
    {TIM_Channel_2, TIMERS_HAL_PWM_HW_UNIT_1}, {TIM_Channel_4, TIMERS_HAL_PWM_HW_UNIT_1},
    {TIM_Channel_1, TIMERS_HAL_PWM_HW_UNIT_0}, {TIM_Channel_3, TIMERS_HAL_PWM_HW_UNIT_0},
    {TIM_Channel_1, TIMERS_HAL_PWM_HW_UNIT_1}, {TIM_Channel_3, TIMERS_HAL_PWM_HW_UNIT_1}
};

void timersHalEnablePwmChannel(TimersHalPwmChannel pwmChannel, bool isEnabled)
{
    TIM_CCxCmd(timChannels[pwmChannel].timer, timChannels[pwmChannel].channel, isEnabled ? TIM_CCx_Enable : TIM_CCx_Disable);
}

void timersHalSetPwmChannelVal(TimersHalPwmChannel pwmChannel, uint32_t value)
{
    switch (pwmChannel) {
    case TIMERS_HAL_PWM_CHANNEL_0 :
        TIMERS_HAL_PWM_HW_UNIT_0->CCR2 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_1 :
        TIMERS_HAL_PWM_HW_UNIT_0->CCR4 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_2 :
        TIMERS_HAL_PWM_HW_UNIT_1->CCR2 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_3 :
        TIMERS_HAL_PWM_HW_UNIT_1->CCR4 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_4 :
        TIMERS_HAL_PWM_HW_UNIT_0->CCR1 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_5 :
        TIMERS_HAL_PWM_HW_UNIT_0->CCR3 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_6 :
        TIMERS_HAL_PWM_HW_UNIT_1->CCR1 = value;
        break;

    case TIMERS_HAL_PWM_CHANNEL_7 :
        TIMERS_HAL_PWM_HW_UNIT_1->CCR3 = value;
        break;

    default :
        break;
    }
}

uint32_t timersHalGetAuxCounterValue(void)
{
    return TIMERS_HAL_AUX_HW_UNIT->CNT;
}

void TIMERS_HAL_AUX_IRQ(void)
{
    TIM_ClearITPendingBit(TIMERS_HAL_AUX_HW_UNIT, TIM_IT_Update);

    if (timerCallback)
        timerCallback();
}

static uint32_t timerDiff(uint32_t x, uint32_t y)
{
    #define KILO    1000
    return y >= x ? (y - x) : (TIMER_HAL_AUX_TICK_RATE / TIMER_HAL_AUX_CB_RATE - x + y);
}

static void delay1ms()
{
    uint32_t timeStamp1 = timersHalGetAuxCounterValue(), timeStamp2 = timeStamp1;
    while (timerDiff(timeStamp1, timeStamp2) < (TIMER_HAL_AUX_TICK_RATE / KILO))
    {
        timeStamp2 = timersHalGetAuxCounterValue();
    }
}

void delayMs(uint32_t delayTime)
{
    while (delayTime--) {
        delay1ms();
    }
}
