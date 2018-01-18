#ifndef __BUTTON_HAL_H__
#define __BUTTON_HAL_H__

#include <stdbool.h>

void buttonHalInit(void);
bool buttonHalGetState(void);

#endif
