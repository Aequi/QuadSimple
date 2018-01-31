#include <stdint.h>
#include <stdbool.h>

#include "Delay.h"
#include "BatteryMonitor.h"
#include "Protocol.h"
#include "RcProcessor.h"
#include "FlightController.h"
#include "SensorSystem.h"

#include "HalCommon.h"
#include "HalPwm.h"

#define TIMER_EVENT_PERIOD  100

static bool isTimerEvent = false;

void delayTimerEventHandler(void)
{
    static uint32_t counter = 0;
    if (counter++ >= TIMER_EVENT_PERIOD) {
        counter = 0;
        isTimerEvent = true;
    }
}

static ProtocolJoystickPacket joysticksStatePacket;
static FlightControllerOrientationEstimate orientationEstimate;
static FlightControllerSetPoint setPoint;
static FlightControllerMotorOutput motorOutput;

void onSensorSystemUpdate(void)
{
    sensorSystemGetCurrentOrientation(&orientationEstimate);
    rcProcessorGetSetPoint(&setPoint, &joysticksStatePacket, &orientationEstimate, PWM_MAX);
    flightControllerUpdate(&setPoint, &orientationEstimate, &motorOutput);
    halPwmSetChannelValue(PWM_CHANNEL_0, motorOutput.motorFrontLeft);
    halPwmSetChannelValue(PWM_CHANNEL_1, motorOutput.motorBackLeft);
    halPwmSetChannelValue(PWM_CHANNEL_2, motorOutput.motorBackRight);
    halPwmSetChannelValue(PWM_CHANNEL_2, motorOutput.motorFrontRight);
}

int main(void)
{
    halCommonInit();
    delayInit(delayTimerEventHandler);
    batteryMonitorInit();
    protocolInit(false);
    sensorSystemCalibrate();
    sensorSystemInit(onSensorSystemUpdate);
    flightControllerInit(SENSOR_SYSTEM_UPDATE_RATE, PWM_MAX);
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

