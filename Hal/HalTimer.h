#ifndef __HAL_TIMER_H__
#define __HAL_TIMER_H__

#include "stdint.h"

typedef void (*TimerCallback)(void);

void halTimerInit(TimerCallback timerCb);

#endif /* __HAL_TIMER_H__ */
