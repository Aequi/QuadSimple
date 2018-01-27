#include "HalTimer.h"
#include "Delay.h"
#include <stddef.h>

static DelayTimerCallback delayTimerCallback = NULL;

static volatile uint32_t delayCounter = 0;

static void timerCb(void)
{
    delayCounter++;
    if (delayTimerCallback != NULL) {
        delayTimerCallback();
    }
}

void delayInit(DelayTimerCallback delayTimerCb)
{
    delayTimerCallback = delayTimerCb;
    halTimerInit(timerCb);
}

void delayMs(uint32_t delay)
{
    delayCounter = 0;
    while (delayCounter < delay) {

    }
}
