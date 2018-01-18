#include "SpiHal.h"
#include "HalCommon.h"
#include "stm32f4xx_conf.h"
#include <stddef.h>

#define SPI_HAL_IMU_SCK_PIN                 GPIO_Pin_13
#define SPI_HAL_IMU_SCK_PIN_SRC             GPIO_PinSource13
#define SPI_HAL_IMU_SCK_PIN_AF              GPIO_AF_SPI2
#define SPI_HAL_IMU_SCK_PORT                GPIOB

#define SPI_HAL_IMU_MISO_PIN                GPIO_Pin_14
#define SPI_HAL_IMU_MISO_PIN_SRC            GPIO_PinSource14
#define SPI_HAL_IMU_MISO_PIN_AF             GPIO_AF_SPI2
#define SPI_HAL_IMU_MISO_PORT               GPIOB

#define SPI_HAL_IMU_MOSI_PIN                GPIO_Pin_15
#define SPI_HAL_IMU_MOSI_PIN_SRC            GPIO_PinSource15
#define SPI_HAL_IMU_MOSI_PIN_AF             GPIO_AF_SPI2
#define SPI_HAL_IMU_MOSI_PORT               GPIOB

#define SPI_HAL_IMU_CS_PIN                  GPIO_Pin_10
#define SPI_HAL_IMU_CS_PIN_SRC              GPIO_PinSource10
#define SPI_HAL_IMU_CS_PORT                 GPIOA

#define SPI_HAL_IMU_INT_PIN                 GPIO_Pin_8
#define SPI_HAL_IMU_INT_PIN_SRC             GPIO_PinSource8
#define SPI_HAL_IMU_INT_PIN_AF              GPIO_AF_TIM1
#define SPI_HAL_IMU_INT_PORT                GPIOA

#define SPI_HAL_IMU_INT_CS_PORT             GPIOA
#define SPI_HAL_IMU_SCK_MISO_MOSI_PORT      GPIOB

#define SPI_HAL_IMU_HW_UNIT                 SPI2
#define SPI_HAL_IMU_DMA_TX_CHANNEL          DMA_Channel_0
#define SPI_HAL_IMU_DMA_RX_CHANNEL          DMA_Channel_0
#define SPI_HAL_IMU_RX_DMA_HW_UNIT          DMA1_Stream3
#define SPI_HAL_IMU_TX_DMA_HW_UNIT          DMA2_Stream6
#define SPI_HAL_IMU_TIM_HW_UNIT             TIM1

#define SPI_HAL_IMU_RX_DMA_IT_TC_FLAG       DMA_IT_TCIF3
#define SPI_HAL_IMU_RX_DMA_IT_HT_FLAG       DMA_IT_HTIF3
#define SPI_HAL_IMU_RX_DMA_IRQ              DMA1_Stream3_IRQHandler
#define SPI_HAL_IMU_RX_DMA_IRQn             DMA1_Stream3_IRQn

#define SPI_HAL_PRE_BYTE_DELAY              10
#define SPI_HAL_BAUD_RATE_RESCALER          SPI_BaudRatePrescaler_8
#define SPI_HAL_BAUD_RATE_RESCALER_VALUE    8
#define SPI_HAL_BIT_PERIOD_COUNT            24


static uint8_t spiHalImuFrameBuffers[2][SPI_HAL_IMU_BUFFER_SIZE + 1];
static const uint8_t spiHalImuTxFrameRegAddress = 0xBB;

static uint8_t *imuBuff = &spiHalImuFrameBuffers[0][1];

static void (*spiHalImuDmaRxCompleteCb)() = NULL;

static void spiHalImuSetCs(bool isSet)
{
    GPIO_WriteBit(SPI_HAL_IMU_INT_CS_PORT, SPI_HAL_IMU_CS_PIN, isSet ? Bit_RESET : Bit_SET);
}

void spiHalImuInit(void (*spiHalImuBuffReadyCb)(void))
{
    if (spiHalImuBuffReadyCb)
        spiHalImuDmaRxCompleteCb = spiHalImuBuffReadyCb;

    GPIO_InitTypeDef gpioInitStructure = {.GPIO_Pin = SPI_HAL_IMU_SCK_PIN | SPI_HAL_IMU_MISO_PIN | SPI_HAL_IMU_MOSI_PIN,
                                          .GPIO_Mode = GPIO_Mode_AF,
                                          .GPIO_Speed = GPIO_Speed_50MHz,
                                          .GPIO_OType = GPIO_OType_PP,
                                          .GPIO_PuPd = GPIO_PuPd_DOWN};

    GPIO_Init(SPI_HAL_IMU_SCK_MISO_MOSI_PORT, &gpioInitStructure);

    GPIO_PinAFConfig(SPI_HAL_IMU_SCK_MISO_MOSI_PORT, SPI_HAL_IMU_SCK_PIN_SRC, SPI_HAL_IMU_SCK_PIN_AF);
    GPIO_PinAFConfig(SPI_HAL_IMU_SCK_MISO_MOSI_PORT, SPI_HAL_IMU_MISO_PIN_SRC, SPI_HAL_IMU_MISO_PIN_AF);
    GPIO_PinAFConfig(SPI_HAL_IMU_SCK_MISO_MOSI_PORT, SPI_HAL_IMU_MOSI_PIN_SRC, SPI_HAL_IMU_MOSI_PIN_AF);

    gpioInitStructure.GPIO_Pin = SPI_HAL_IMU_INT_PIN;

    GPIO_Init(SPI_HAL_IMU_INT_CS_PORT, &gpioInitStructure);

    GPIO_PinAFConfig(SPI_HAL_IMU_INT_CS_PORT, SPI_HAL_IMU_INT_PIN_SRC, SPI_HAL_IMU_INT_PIN_AF);

    gpioInitStructure.GPIO_Pin = SPI_HAL_IMU_CS_PIN;
    gpioInitStructure.GPIO_Mode = GPIO_Mode_OUT;

    GPIO_Init(SPI_HAL_IMU_INT_CS_PORT, &gpioInitStructure);

    spiHalImuSetCs(false);

    SPI_InitTypeDef spiInitStructure = {.SPI_Direction = SPI_Direction_2Lines_FullDuplex,
                                        .SPI_Mode = SPI_Mode_Master,
                                        .SPI_DataSize = SPI_DataSize_8b,
                                        .SPI_CPOL = SPI_CPOL_Low,
                                        .SPI_CPHA = SPI_CPHA_1Edge,
                                        .SPI_NSS = SPI_NSS_Soft,
                                        .SPI_BaudRatePrescaler = SPI_HAL_BAUD_RATE_RESCALER,
                                        .SPI_FirstBit = SPI_FirstBit_MSB,
                                        .SPI_CRCPolynomial = 0};
    SPI_Init(SPI_HAL_IMU_HW_UNIT, &spiInitStructure);
    SPI_CalculateCRC(SPI_HAL_IMU_HW_UNIT, DISABLE);
    SPI_Cmd(SPI_HAL_IMU_HW_UNIT, ENABLE);
}

void SPI_HAL_IMU_RX_DMA_IRQ(void)
{
    spiHalImuSetCs(false);
    if (DMA_GetITStatus(SPI_HAL_IMU_RX_DMA_HW_UNIT, SPI_HAL_IMU_RX_DMA_IT_HT_FLAG)) {
        DMA_ClearITPendingBit(SPI_HAL_IMU_RX_DMA_HW_UNIT, SPI_HAL_IMU_RX_DMA_IT_HT_FLAG);
        imuBuff = &spiHalImuFrameBuffers[0][1];
    } else if (DMA_GetITStatus(SPI_HAL_IMU_RX_DMA_HW_UNIT, SPI_HAL_IMU_RX_DMA_IT_TC_FLAG)) {
        DMA_ClearITPendingBit(SPI_HAL_IMU_RX_DMA_HW_UNIT, SPI_HAL_IMU_RX_DMA_IT_TC_FLAG);
        imuBuff = &spiHalImuFrameBuffers[1][1];
    }
    spiHalImuSetCs(true);

    if (spiHalImuDmaRxCompleteCb)
        spiHalImuDmaRxCompleteCb();

}

uint8_t *spiHalImuGetBuffer(void)
{
    return imuBuff;
}

void spiHalImuReadOutStart(void)
{
    SPI_I2S_DMACmd(SPI_HAL_IMU_HW_UNIT, SPI_I2S_DMAReq_Rx, ENABLE);

    spiHalImuSetCs(true);

    RCC_ClocksTypeDef rccClocks;
    RCC_GetClocksFreq(&rccClocks);

    DMA_InitTypeDef dmaInitStructure;

    /*Configure DMA for spi rx*/
    dmaInitStructure.DMA_BufferSize = sizeof(spiHalImuFrameBuffers);
    dmaInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInitStructure.DMA_Memory0BaseAddr = (uint32_t) spiHalImuFrameBuffers;
    dmaInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStructure.DMA_Mode = DMA_Mode_Circular;
    dmaInitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI_HAL_IMU_HW_UNIT->DR;
    dmaInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    dmaInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dmaInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dmaInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInitStructure.DMA_Channel = SPI_HAL_IMU_DMA_RX_CHANNEL;
    DMA_DoubleBufferModeCmd(SPI_HAL_IMU_RX_DMA_HW_UNIT, DISABLE);
    DMA_Init(SPI_HAL_IMU_RX_DMA_HW_UNIT, &dmaInitStructure);
    DMA_ITConfig(SPI_HAL_IMU_RX_DMA_HW_UNIT, DMA_IT_TC, ENABLE);
    DMA_ITConfig(SPI_HAL_IMU_RX_DMA_HW_UNIT, DMA_IT_HT, ENABLE);
    DMA_Cmd(SPI_HAL_IMU_RX_DMA_HW_UNIT, ENABLE);
    NVIC_EnableIRQ(SPI_HAL_IMU_RX_DMA_IRQn);
    TIM_TimeBaseInitTypeDef timBaseInitStructure;
    TIM_TimeBaseStructInit(&timBaseInitStructure);
    timBaseInitStructure.TIM_Prescaler = 0;
    timBaseInitStructure.TIM_RepetitionCounter = SPI_HAL_IMU_BUFFER_SIZE + sizeof(spiHalImuTxFrameRegAddress) - 1;
    timBaseInitStructure.TIM_Period = rccClocks.SYSCLK_Frequency / (rccClocks.PCLK1_Frequency /
                                      SPI_HAL_BAUD_RATE_RESCALER_VALUE) * SPI_HAL_BIT_PERIOD_COUNT - 1;
    TIM_TimeBaseInit(SPI_HAL_IMU_TIM_HW_UNIT, &timBaseInitStructure);

    TIM_SelectOnePulseMode(SPI_HAL_IMU_TIM_HW_UNIT, TIM_OPMode_Single);

    TIM_OCInitTypeDef timOcInitStructure;
    TIM_OCStructInit(&timOcInitStructure);

    timOcInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    timOcInitStructure.TIM_Pulse = SPI_HAL_PRE_BYTE_DELAY;
    TIM_OC3Init(SPI_HAL_IMU_TIM_HW_UNIT, &timOcInitStructure);
    TIM_OC3PreloadConfig(SPI_HAL_IMU_TIM_HW_UNIT, TIM_OCPreload_Enable);
    TIM_CCxCmd(SPI_HAL_IMU_TIM_HW_UNIT, TIM_Channel_3, TIM_CCx_Enable);
    TIM_OC3FastConfig(SPI_HAL_IMU_TIM_HW_UNIT, TIM_OCFast_Disable);
    TIM_SetCompare3(SPI_HAL_IMU_TIM_HW_UNIT, SPI_HAL_PRE_BYTE_DELAY);
    TIM_CtrlPWMOutputs(SPI_HAL_IMU_TIM_HW_UNIT, DISABLE);
    TIM_DMACmd(SPI_HAL_IMU_TIM_HW_UNIT, TIM_DMA_CC3, ENABLE);

    /*Configure DMA for DUMMY tim1 ch3  tx*/
    dmaInitStructure.DMA_BufferSize = sizeof(spiHalImuTxFrameRegAddress);
    dmaInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dmaInitStructure.DMA_Memory0BaseAddr = (uint32_t) &spiHalImuTxFrameRegAddress;
    dmaInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    dmaInitStructure.DMA_Mode = DMA_Mode_Circular;
    dmaInitStructure.DMA_PeripheralBaseAddr = (uint32_t) &SPI_HAL_IMU_HW_UNIT->DR;
    dmaInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    dmaInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dmaInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dmaInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInitStructure.DMA_Channel = SPI_HAL_IMU_DMA_TX_CHANNEL;
    DMA_DoubleBufferModeCmd(SPI_HAL_IMU_TX_DMA_HW_UNIT, DISABLE);
    DMA_Init(SPI_HAL_IMU_TX_DMA_HW_UNIT, &dmaInitStructure);
    DMA_Cmd(SPI_HAL_IMU_TX_DMA_HW_UNIT, ENABLE);

    TIM_ICInitTypeDef timIcInitStructure;
    timIcInitStructure.TIM_Channel = TIM_Channel_1;
    timIcInitStructure.TIM_ICFilter = 0;
    timIcInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    timIcInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    timIcInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(SPI_HAL_IMU_TIM_HW_UNIT, &timIcInitStructure);
    TIM_SelectInputTrigger(SPI_HAL_IMU_TIM_HW_UNIT, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(SPI_HAL_IMU_TIM_HW_UNIT, TIM_SlaveMode_Trigger);
}

static void spiHalImuXchg(const uint8_t outputData[], uint8_t inputData[], uint32_t dataSize)
{
    uint32_t byteCounter = 0;

    for (byteCounter = 0; byteCounter < dataSize; byteCounter++) {
        while (!SPI_I2S_GetFlagStatus(SPI_HAL_IMU_HW_UNIT, SPI_I2S_FLAG_TXE)) {

        }

        SPI_I2S_SendData(SPI_HAL_IMU_HW_UNIT, outputData[byteCounter]);

        while (!SPI_I2S_GetFlagStatus(SPI_HAL_IMU_HW_UNIT, SPI_I2S_FLAG_RXNE)) {

        }

        inputData[byteCounter] = SPI_I2S_ReceiveData(SPI_HAL_IMU_HW_UNIT);
    }

    while (SPI_I2S_GetFlagStatus(SPI_HAL_IMU_HW_UNIT, SPI_I2S_FLAG_BSY)) {

    }
}

uint8_t spiHalImuReadReg(const uint8_t reg)
{
    const uint8_t txBlock[] = {reg | 0x80, 0};
    uint8_t rxBlock[] = {0, 0};
    spiHalImuSetCs(true);
    spiHalImuXchg(txBlock, rxBlock, sizeof(txBlock));
    spiHalImuSetCs(false);
    return rxBlock[1];
}

void spiHalImuWriteReg(const uint8_t reg, const uint8_t value)
{
    const uint8_t txBlock[] = {reg & ~0x80, value};
    uint8_t rxBlock[] = {0, 0};
    spiHalImuSetCs(true);
    spiHalImuXchg(txBlock, rxBlock, sizeof(txBlock));
    spiHalImuSetCs(false);
}
