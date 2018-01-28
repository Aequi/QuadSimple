#include "HalCommon.h"
#include "Delay.h"
#include "Protocol.h"
#include "BatteryMonitor.h"
#include "HalPwm.h"

#define TIMER_EVENT_PERIOD  100

static ProtocolJoystickPacket joysticksStatePacket;
static bool isTimerEvent = false;

void delayTimerEventHandler(void)
{
    static uint32_t counter = 0;
    if (counter++ >= TIMER_EVENT_PERIOD) {
        counter = 0;
        isTimerEvent = true;
    }
}

int main(void)
{
    halCommonInit();
    delayInit(delayTimerEventHandler);
    batteryMonitorInit();
    protocolInit(false);

    for (;;) {
        protocolProcess();
        if (protocolIsPacketReady()) {
            joysticksStatePacket = protocolGetJoystickValues();
        }

        if (isTimerEvent) {
            batteryMonitorProcess();
            uint8_t soc = batteryMonitorGetSoc();
            protocolSendBatterySoc(soc);
            isTimerEvent = false;
        }
    }
}

