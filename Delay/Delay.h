#ifndef __DELAY_H__
#define __DELAY_H__

#include "stdint.h"

typedef void (*DelayTimerCallback)(void);

void delayInit(DelayTimerCallback delayTimerCb);
void delayMs(uint32_t delay);

#endif /* __DELAY_H__ */
