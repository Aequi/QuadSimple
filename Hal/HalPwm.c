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

    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH0_PIN_SOURCE, GPIO_AF_1);
    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH1_PIN_SOURCE, GPIO_AF_1);
    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH2_PIN_SOURCE, GPIO_AF_2);
    GPIO_PinAFConfig(HAL_PWM_PORT, HAL_PWM_CH3_PIN_SOURCE, GPIO_AF_2);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = PWM_MAX;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC1FastConfig(TIM2, TIM_OCFast_Disable);

    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2FastConfig(TIM2, TIM_OCFast_Disable);

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC1FastConfig(TIM3, TIM_OCFast_Disable);

    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2FastConfig(TIM3, TIM_OCFast_Disable);

    TIM_Cmd(TIM2, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    halPwmEnable(PWM_CHANNEL_0, false);
    halPwmEnable(PWM_CHANNEL_1, false);
    halPwmEnable(PWM_CHANNEL_2, false);
    halPwmEnable(PWM_CHANNEL_3, false);
}

static const struct timerChannels {
    uint16_t channel;
    TIM_TypeDef *timer;
} timChannels[] = {
    {TIM_Channel_2, TIM3},
    {TIM_Channel_1, TIM3},
    {TIM_Channel_2, TIM2},
    {TIM_Channel_1, TIM2},
};

void halPwmEnable(PwmChannel pwmChannel, bool isEnabled)
{
    TIM_CCxCmd(timChannels[pwmChannel].timer, timChannels[pwmChannel].channel, isEnabled ? TIM_CCx_Enable : TIM_CCx_Disable);
}

void halPwmSetChannelValue(PwmChannel pwmChannel, uint32_t value)
{
    value /= 3;
    switch (pwmChannel) {
    case PWM_CHANNEL_0 :
        TIM3->CCR2 = value;
        break;

    case PWM_CHANNEL_1 :
        TIM3->CCR1 = value;
        break;

    case PWM_CHANNEL_2 :
        TIM2->CCR2 = value;
        break;

    case PWM_CHANNEL_3 :
        TIM2->CCR1 = value;
        break;

    }
}
