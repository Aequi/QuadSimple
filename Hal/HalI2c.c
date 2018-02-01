#include "HalI2c.h"
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

#define I2C_PERIPH_IMU_ADDRESS                          0x78

static void i2cInit(void)
{
    GPIO_InitTypeDef gpioInitStructure = {
        .GPIO_Pin = HAL_I2C_IMU_SDA_PIN | HAL_I2C_IMU_SDA_PIN,
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
}

static void i2cWrite(uint8_t chipAddress, uint8_t registerAddress, uint8_t data)
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

void halI2cImuInit(void)
{
    i2cInit();

}

void EXTI0_1_IRQHandler(void)
{

}
