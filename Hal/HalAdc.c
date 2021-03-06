#include "HalAdc.h"
#include "HalCommon.h"
#include "stm32f0xx_conf.h"

#include <stddef.h>

#define HAL_ADC_BATTERY_PIN     GPIO_Pin_5
#define HAL_ADC_BATTERY_PORT    GPIOA

static uint16_t adcDataArray[2][ADC_BUFF_SIZE * ADC_CHANNEL_COUNT];

static HalAdcDataReady halAdcDataReadyCallback;

static void halAdcDmaHandler(bool isHalf)
{
    if (isHalf) {
        if (halAdcDataReadyCallback != NULL)
            halAdcDataReadyCallback((const uint16_t *) adcDataArray[0], sizeof(adcDataArray[0]) / sizeof(adcDataArray[0][0]));
    } else {
        if (halAdcDataReadyCallback != NULL)
            halAdcDataReadyCallback((const uint16_t *) adcDataArray[1], sizeof(adcDataArray[1]) / sizeof(adcDataArray[0][0]));
    }
}

void halAdcInit(HalAdcDataReady halAdcDataReadyCb)
{
    halAdcDataReadyCallback = halAdcDataReadyCb;
    GPIO_InitTypeDef gpioInitStructure = {
        .GPIO_Pin =  HAL_ADC_BATTERY_PIN,
        .GPIO_Mode = GPIO_Mode_AN,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_UP
    };

    GPIO_Init(HAL_ADC_BATTERY_PORT, &gpioInitStructure);

    ADC_ClockModeConfig(ADC1, ADC_ClockMode_AsynClk);
    ADC_InitTypeDef adcInitStructure;
    ADC_StructInit(&adcInitStructure);
    adcInitStructure.ADC_ContinuousConvMode = ENABLE;
    adcInitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    adcInitStructure.ADC_Resolution = ADC_Resolution_12b;
    adcInitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &adcInitStructure);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_239_5Cycles);
    ADC_ContinuousModeCmd(ADC1, ENABLE);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
    ADC_Cmd(ADC1, ENABLE);
    ADC_StartOfConversion(ADC1);

    dmaCh1Cb = halAdcDmaHandler;
}

void halAdcStartStop(bool isStart)
{
    DMA_InitTypeDef dmaInitStructure;
    if (isStart) {
        dmaInitStructure.DMA_BufferSize = sizeof(adcDataArray) / sizeof(adcDataArray[0][0]);
        dmaInitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        dmaInitStructure.DMA_M2M = DMA_M2M_Disable;
        dmaInitStructure.DMA_MemoryBaseAddr = (uint32_t) adcDataArray;
        dmaInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        dmaInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmaInitStructure.DMA_Mode = DMA_Mode_Circular;
        dmaInitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
        dmaInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        dmaInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmaInitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_Init(DMA1_Channel1, &dmaInitStructure);
        DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
        DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
        DMA_Cmd(DMA1_Channel1, ENABLE);
    } else {
        DMA_Cmd(DMA1_Channel1, DISABLE);
    }
}
