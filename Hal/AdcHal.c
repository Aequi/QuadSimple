#include "stm32f4xx_conf.h"
#include "AdcHal.h"

#define ADC_HAL_PIN                     GPIO_Pin_5
#define ADC_HAL_PORT                    GPIOA
#define ADC_HAL_CHANNEL                 5
#define ADC_HAL_HW_UNIT                 ADC1
#define ADC_HAL_CHANNEL_RANK            1
#define ADC_HAL_NUMBER_OF_CONVERSIONS   1

void adcHalInit(void)
{
    GPIO_InitTypeDef gpioInitStructure = {.GPIO_Pin = ADC_HAL_PIN,
                                          .GPIO_Mode = GPIO_Mode_AIN,
                                          .GPIO_Speed = GPIO_Speed_50MHz,
                                          .GPIO_OType = GPIO_OType_PP,
                                          .GPIO_PuPd = GPIO_PuPd_NOPULL};

    GPIO_Init(ADC_HAL_PORT, &gpioInitStructure);

    ADC_InitTypeDef adcInitStructure;
    ADC_StructInit(&adcInitStructure);
    adcInitStructure.ADC_Resolution = ADC_Resolution_12b;
    adcInitStructure.ADC_ContinuousConvMode = DISABLE;
    adcInitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    adcInitStructure.ADC_NbrOfConversion = ADC_HAL_NUMBER_OF_CONVERSIONS;
    adcInitStructure.ADC_ScanConvMode = DISABLE;

    ADC_Init(ADC_HAL_HW_UNIT, &adcInitStructure);
    ADC_Cmd(ADC_HAL_HW_UNIT, ENABLE);

    ADC_RegularChannelConfig(ADC_HAL_HW_UNIT, ADC_HAL_CHANNEL, ADC_HAL_CHANNEL_RANK, ADC_SampleTime_480Cycles);

    ADC_SoftwareStartConv(ADC_HAL_HW_UNIT);
}

uint32_t adcHalGetValue(void)
{
    static uint32_t adcConversionValue = 0;
    adcConversionValue = ADC_GetConversionValue(ADC_HAL_HW_UNIT);
    ADC_SoftwareStartConv(ADC_HAL_HW_UNIT);

    return adcConversionValue;
}
