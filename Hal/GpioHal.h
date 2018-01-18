#ifndef __GPIO_HAL_H__
#define __GPIO_HAL_H__

#include <stdbool.h>

void gpioHalInit(void);
void gpioHalEnableBt(bool isEnabled);
void gpioHalEnableBtAt(bool isEnabled);
void gpioHalEnablePower(bool isEnabled);
bool gpioHalGetBtConState(void);


#endif
