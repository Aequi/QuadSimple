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

static uint8_t halI2cReadBuffer[HAL_I2C_READ_BUFF_SIZE];
static uint32_t bytesToRead;

void EXTI0_1_IRQHandler(void)
{
    if (halIntPinEventCallback != NULL) {
        halIntPinEventCallback();
    }
}

void I2C1_IRQHandler(void)
{

}

void i2cTxDmaCb(bool isHalf)
{

}

void i2cRxDmaCb(bool isHalf)
{
    if (halI2cDmaReadCallback != NULL) {
        halI2cDmaReadCallback();
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
    i2cInitStructure.I2C_Timing = 0x00000102;
    i2cInitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInitStructure.I2C_OwnAddress1 = I2C_PERIPH_OWN_ADDRESS;
    i2cInitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_Init(I2C_PERIPH_HWUNIT, &i2cInitStructure);
    I2C_AcknowledgeConfig(I2C_PERIPH_HWUNIT, ENABLE);
    I2C_StretchClockCmd(I2C_PERIPH_HWUNIT, ENABLE);
    I2C_Cmd(I2C_PERIPH_HWUNIT, ENABLE);
    dmaCh2Cb = i2cTxDmaCb;
    dmaCh3Cb = i2cRxDmaCb;
}

void i2cWrite(uint8_t chipAddress, uint8_t registerAddress, uint8_t data)
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

uint8_t i2cRead(uint8_t chipAddress, uint8_t registerAddress)
{
    while (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_BUSY));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C_PERIPH_HWUNIT, registerAddress);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Read);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	uint8_t data = I2C_ReceiveData(I2C_PERIPH_HWUNIT);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_RXNE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 0, I2C_AutoEnd_Mode,  I2C_Generate_Stop);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);

    return data;
}

void halI2cInit(HalI2cDmaReadCallback halI2cDmaReadCb, HalIntPinEventCallback halIntPinEventCb)
{
    halI2cDmaReadCallback = halI2cDmaReadCb;
    halIntPinEventCallback = halIntPinEventCb;
    i2cInit();
}

uint8_t i2cReadWithDma(uint8_t chipAddress, uint8_t startRegisterAddress, uint32_t length)
{
    bytesToRead = length;
}

uint8_t *i2cGetReadBuffer(uint32_t *bufferSize)
{
    if (bufferSize == NULL) {
        *bufferSize = bytesToRead;
    }

    return halI2cReadBuffer;
}
