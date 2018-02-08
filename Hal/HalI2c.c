#include "HalI2c.h"
#include "HalCommon.h"
#include "stm32f0xx_conf.h"

#include <stddef.h>

#define HAL_I2C_IMU_SDA_PIN                             GPIO_Pin_10
#define HAL_I2C_IMU_SDA_PIN_SOURCE                      GPIO_PinSource10

#define HAL_I2C_IMU_SCL_PIN                             GPIO_Pin_9
#define HAL_I2C_IMU_SCL_PIN_SOURCE                      GPIO_PinSource9

#define HAL_INT_IMU_PIN                                 GPIO_Pin_1
#define HAL_INT_IMU_PIN_SOURCE                          GPIO_PinSource1

#define HAL_I2C_IMU_PORT                                GPIOA
#define HAL_INT_IMU_PORT                                GPIOB

#define I2C_PERIPH_HWUNIT                               I2C1

#define I2C_PERIPH_OWN_ADDRESS                          1u

#define HAL_I2C_READ_BUFF_SIZE                          128

static HalI2cDmaReadCallback halI2cDmaReadCallback;
static HalIntPinEventCallback halIntPinEventCallback;

static uint8_t halI2cReadBuffer[HAL_I2C_READ_BUFF_SIZE], halI2cRegAddress, halI2cChipAddress;
static uint32_t bytesToRead;

void EXTI0_1_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line1);
    if (halIntPinEventCallback != NULL) {
        halIntPinEventCallback();
    }
}

static enum {
    I2C_STATE_STOP,
    I2C_STATE_TX_START,
    I2C_STATE_TX,
    I2C_STATE_RX,
} i2cState = I2C_STATE_STOP;

volatile static int16_t xAcc = 0;
volatile static int16_t yAcc = 0;

void I2C1_IRQHandler(void)
{
    switch (i2cState) {
    case I2C_STATE_TX_START :
        if (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_IT_TXIS)) {
            I2C_ClearITPendingBit(I2C_PERIPH_HWUNIT, I2C_IT_TXIS);
        }
        I2C_ITConfig(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS, DISABLE);
        I2C_SendData(I2C_PERIPH_HWUNIT, halI2cRegAddress);
        I2C_ITConfig(I2C_PERIPH_HWUNIT, I2C_IT_TC, ENABLE);
        i2cState = I2C_STATE_TX;
        break;

    case I2C_STATE_TX :
        if (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TCR)) {
            I2C_ClearITPendingBit(I2C_PERIPH_HWUNIT, I2C_IT_TC);
        }
        I2C_ITConfig(I2C_PERIPH_HWUNIT, I2C_IT_TC, DISABLE);
        I2C_ITConfig(I2C_PERIPH_HWUNIT, I2C_ISR_STOPF, ENABLE);
        I2C_DMACmd(I2C_PERIPH_HWUNIT, I2C_DMAReq_Rx, ENABLE);
        DMA_InitTypeDef dmaInitStructure;
        dmaInitStructure.DMA_BufferSize = bytesToRead;
        dmaInitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        dmaInitStructure.DMA_M2M = DMA_M2M_Disable;
        dmaInitStructure.DMA_MemoryBaseAddr = (uint32_t) halI2cReadBuffer;
        dmaInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        dmaInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        dmaInitStructure.DMA_Mode = DMA_Mode_Normal;
        dmaInitStructure.DMA_PeripheralBaseAddr = (uint32_t) &I2C1->RXDR;
        dmaInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        dmaInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        dmaInitStructure.DMA_Priority = DMA_Priority_Medium;
        DMA_Init(DMA1_Channel3, &dmaInitStructure);
        DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
        DMA_Cmd(DMA1_Channel3, ENABLE);
        I2C_TransferHandling(I2C_PERIPH_HWUNIT, halI2cChipAddress, bytesToRead, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
        i2cState = I2C_STATE_RX;
        break;

    case I2C_STATE_RX :
        if (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_IT_STOPF)) {
            I2C_ClearITPendingBit(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);
        }
        I2C_DMACmd(I2C_PERIPH_HWUNIT, I2C_DMAReq_Rx, DISABLE);
        DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, DISABLE);
        DMA_Cmd(DMA1_Channel3, DISABLE);
        i2cState = I2C_STATE_STOP;
        xAcc = (int16_t) (((uint16_t) halI2cReadBuffer[0] << 8) | ((uint16_t) halI2cReadBuffer[1] << 0));
        yAcc = (int16_t) (((uint16_t) halI2cReadBuffer[2] << 8) | ((uint16_t) halI2cReadBuffer[3] << 0));
        break;

    default :
        break;
    }
}

void i2cRxDmaCb(bool isHalf)
{
    if (!isHalf) {
        if (halI2cDmaReadCallback != NULL) {
            halI2cDmaReadCallback();
        }
    }
}

static void i2cInit(void)
{
    GPIO_InitTypeDef gpioInitStructure = {
        .GPIO_Pin = HAL_I2C_IMU_SDA_PIN | HAL_I2C_IMU_SCL_PIN,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_PuPd = GPIO_PuPd_UP
    };

    GPIO_Init(HAL_I2C_IMU_PORT, &gpioInitStructure);

    gpioInitStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioInitStructure.GPIO_Pin = HAL_INT_IMU_PIN;
    gpioInitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(HAL_INT_IMU_PORT, &gpioInitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

    EXTI_InitTypeDef extiInitStructure = {
        .EXTI_Line = EXTI_Line1,
        .EXTI_Mode = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Falling,
        .EXTI_LineCmd = ENABLE,
    };

    EXTI_Init(&extiInitStructure);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    GPIO_PinAFConfig(HAL_I2C_IMU_PORT, HAL_I2C_IMU_SDA_PIN_SOURCE, GPIO_AF_4);
    GPIO_PinAFConfig(HAL_I2C_IMU_PORT, HAL_I2C_IMU_SCL_PIN_SOURCE, GPIO_AF_4);

    I2C_InitTypeDef i2cInitStructure;
    I2C_StructInit(&i2cInitStructure);
    i2cInitStructure.I2C_Mode = I2C_Mode_I2C;
    i2cInitStructure.I2C_Timing = 0x00000808;
    i2cInitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInitStructure.I2C_OwnAddress1 = I2C_PERIPH_OWN_ADDRESS;
    i2cInitStructure.I2C_Ack = I2C_Ack_Enable;
    i2cInitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    i2cInitStructure.I2C_DigitalFilter = 0x0F;

    I2C_Init(I2C_PERIPH_HWUNIT, &i2cInitStructure);
    I2C_AcknowledgeConfig(I2C_PERIPH_HWUNIT, ENABLE);
    I2C_StretchClockCmd(I2C_PERIPH_HWUNIT, ENABLE);
    I2C_Cmd(I2C_PERIPH_HWUNIT, ENABLE);
    dmaCh3Cb = i2cRxDmaCb;
}

void halI2cWrite(uint8_t chipAddress, uint8_t registerAddress, uint8_t data)
{
    while (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_BUSY));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C_PERIPH_HWUNIT, registerAddress);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

	I2C_SendData(I2C_PERIPH_HWUNIT, data);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 0, I2C_AutoEnd_Mode,  I2C_Generate_Stop);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);
}

uint8_t halI2cRead(uint8_t chipAddress, uint8_t registerAddress)
{
    while (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_BUSY));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C_PERIPH_HWUNIT, registerAddress);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Read);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_RXNE));
	uint8_t data = I2C_ReceiveData(I2C_PERIPH_HWUNIT);

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 0, I2C_AutoEnd_Mode,  I2C_Generate_Stop);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);

    return data;
}

void halI2cReadBuf(uint8_t chipAddress, uint8_t registerAddress, uint8_t data[], uint32_t dataLength)
{
    while (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_BUSY));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C_PERIPH_HWUNIT, registerAddress);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, dataLength, I2C_SoftEnd_Mode, I2C_Generate_Start_Read);

    while (dataLength-- > 0) {
        while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_RXNE));
        *data++ = I2C_ReceiveData(I2C_PERIPH_HWUNIT);
    }

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 0, I2C_AutoEnd_Mode,  I2C_Generate_Stop);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);
}

void halI2cInit(HalI2cDmaReadCallback halI2cDmaReadCb, HalIntPinEventCallback halIntPinEventCb)
{
    halI2cDmaReadCallback = halI2cDmaReadCb;
    halIntPinEventCallback = halIntPinEventCb;
    i2cInit();
}

void halI2cReadWithDma(uint8_t chipAddress, uint8_t startRegisterAddress, uint32_t length)
{
    if (i2cState == I2C_STATE_STOP) {
        bytesToRead = length;
        halI2cRegAddress = startRegisterAddress;
        halI2cChipAddress = chipAddress;
        i2cState = I2C_STATE_TX_START;
        NVIC_EnableIRQ(I2C1_IRQn);
        I2C_ITConfig(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS, ENABLE);
        I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    }
}

uint8_t *halI2cGetReadBuffer(uint32_t *bufferSize)
{
    if (bufferSize != NULL) {
        *bufferSize = bytesToRead;
    }

    return halI2cReadBuffer;
}
