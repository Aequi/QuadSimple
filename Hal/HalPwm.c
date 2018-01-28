#include "HalPwm.h"
#include "stm32f0xx_conf.h"

#define HAL_PWM_CH0_PIN             GPIO_Pin_6
#define HAL_PWM_CH0_PIN_SOURCE      GPIO_PinSource6
#define HAL_PWM_CH1_PIN             GPIO_Pin_7
#define HAL_PWM_CH1_PIN_SOURCE      GPIO_PinSource7
#define HAL_PWM_CH2_PIN             GPIO_Pin_0
#define HAL_PWM_CH2_PIN_SOURCE      GPIO_PinSource0
#define HAL_PWM_CH3_PIN             GPIO_Pin_1
#define HAL_PWM_CH3_PIN_SOURCE      GPIO_PinSource1

#define HAL_PWM_PORT                GPIOA

void halPwmInit(void)
{
    GPIO_InitTypeDef gpioInitStructure = {
        .GPIO_Pin = HAL_PWM_CH0_PIN | HAL_PWM_CH1_PIN | HAL_PWM_CH2_PIN | HAL_PWM_CH3_PIN,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_DOWN
    };

    GPIO_Init(HAL_PWM_PORT, &gpioInitStructure);

    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH0_PIN_SOURCE, GPIO_AF_2);
    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH1_PIN_SOURCE, GPIO_AF_2);
    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH2_PIN_SOURCE, GPIO_AF_2);
    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH3_PIN_SOURCE, GPIO_AF_2);

    RCC_ClocksTypeDef rccClocks;

    RCC_GetClocksFreq(&rccClocks);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = rccClocks.PCLK_Frequency / (PWM_MAX + 1) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    TIM_CtrlPWMOutputs(TIM2, DISABLE);
    TIM_CtrlPWMOutputs(TIM3, DISABLE);
}

void halPwmSetChannelValue(PwmChannel pwmChannel, uint32_t value)
{
    switch (pwmChannel) {
    case PWM_CHANNEL_0 :
        TIM2->CCR1 = value;
        break;

    case PWM_CHANNEL_1 :
        TIM2->CCR2 = value;
        break;

    case PWM_CHANNEL_2 :
        TIM3->CCR3 = value;
        break;

    case PWM_CHANNEL_3 :
        TIM3->CCR4 = value;
        break;

    }
}

void halPwmEnable(void)
{
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
}

void halPwmDisable(void)
{
    TIM_CtrlPWMOutputs(TIM2, DISABLE);
    TIM_CtrlPWMOutputs(TIM3, DISABLE);
}
