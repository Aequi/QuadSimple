#include "GpioHal.h"
#include "stm32f4xx_conf.h"

#define HAL_GPIO_BT_ENABLE_PIN      GPIO_Pin_1
#define HAL_GPIO_BT_ENABLE_PORT     GPIOA

#define HAL_GPIO_BT_MODE_PIN        GPIO_Pin_10
#define HAL_GPIO_BT_MODE_PORT       GPIOB

#define HAL_GPIO_BT_STATE_PIN       GPIO_Pin_12
#define HAL_GPIO_BT_STATE_PORT      GPIOB

#define HAL_GPIO_POWER_ENABLE_PIN   GPIO_Pin_13
#define HAL_GPIO_POWER_ENABLE_PORT  GPIOC

void gpioHalInit()
{
    GPIO_InitTypeDef gpioInitStructure = {.GPIO_Pin = HAL_GPIO_BT_ENABLE_PIN,
                                          .GPIO_Mode = GPIO_Mode_OUT,
                                          .GPIO_Speed = GPIO_Speed_50MHz,
                                          .GPIO_OType = GPIO_OType_PP,
                                          .GPIO_PuPd = GPIO_PuPd_DOWN };

    GPIO_Init(HAL_GPIO_BT_ENABLE_PORT, &gpioInitStructure);

    gpioInitStructure.GPIO_Pin = HAL_GPIO_BT_MODE_PIN;
    GPIO_Init(HAL_GPIO_BT_MODE_PORT, &gpioInitStructure);

    gpioInitStructure.GPIO_Pin = HAL_GPIO_POWER_ENABLE_PIN;
    GPIO_Init(HAL_GPIO_POWER_ENABLE_PORT, &gpioInitStructure);

    gpioInitStructure.GPIO_Pin = HAL_GPIO_BT_STATE_PIN;
    gpioInitStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioInitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    GPIO_Init(HAL_GPIO_BT_STATE_PORT, &gpioInitStructure);
}

void gpioHalEnableBt(bool isEnabled)
{
    GPIO_WriteBit(HAL_GPIO_BT_ENABLE_PORT, HAL_GPIO_BT_ENABLE_PIN, isEnabled ? Bit_SET : Bit_RESET);
}

void gpioHalEnableBtAt(bool isEnabled)
{
    GPIO_WriteBit(HAL_GPIO_BT_MODE_PORT, HAL_GPIO_BT_MODE_PIN, isEnabled ? Bit_SET : Bit_RESET);
}

bool gpioHalGetBtConState(void)
{
    return GPIO_ReadInputDataBit(HAL_GPIO_BT_STATE_PORT, HAL_GPIO_BT_STATE_PIN);
}

void gpioHalEnablePower(bool isEnabled)
{
    GPIO_WriteBit(HAL_GPIO_POWER_ENABLE_PORT, HAL_GPIO_POWER_ENABLE_PIN, isEnabled ? Bit_SET : Bit_RESET);
}
