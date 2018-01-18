#include "ButtonHal.h"
#include "stm32f4xx_conf.h"

#define BUTTON_HAL_PIN          GPIO_Pin_14
#define BUTTON_HAL_PORT         GPIOC

void buttonHalInit(void)
{
    GPIO_InitTypeDef gpioInitStructure = {.GPIO_Pin = BUTTON_HAL_PIN,
                                          .GPIO_Mode = GPIO_Mode_IN,
                                          .GPIO_Speed = GPIO_Speed_50MHz,
                                          .GPIO_OType = GPIO_OType_PP,
                                          .GPIO_PuPd = GPIO_PuPd_DOWN};

    GPIO_Init(BUTTON_HAL_PORT, &gpioInitStructure);
}


bool buttonHalGetState(void)
{
    return GPIO_ReadInputDataBit(BUTTON_HAL_PORT, BUTTON_HAL_PIN);
}
