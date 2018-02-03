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

#define TIMER_EVENT_PERIOD          100
#define RC_ALIVE_TIMEOUT            1000
#define SENSOR_STARTUP_TIMEOUT      1000

static bool isTimerEvent = false;
static uint32_t rcAliveTimer = 0;

void delayTimerEventHandler(void)
{
    if (rcAliveTimer++ >= RC_ALIVE_TIMEOUT) {
        rcAliveTimer = RC_ALIVE_TIMEOUT;
    }

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

    if (rcAliveTimer >= RC_ALIVE_TIMEOUT) {
        rcProcessorGetIdleSetPoint(&setPoint, &orientationEstimate);
    } else {
        rcProcessorGetSetPoint(&setPoint, &joysticksStatePacket, &orientationEstimate);
    }

    flightControllerUpdate(&setPoint, &orientationEstimate, &motorOutput);
    halPwmSetChannelValue(PWM_CHANNEL_0, motorOutput.motorFrontLeft);
    halPwmSetChannelValue(PWM_CHANNEL_1, motorOutput.motorBackLeft);
    halPwmSetChannelValue(PWM_CHANNEL_2, motorOutput.motorBackRight);
    halPwmSetChannelValue(PWM_CHANNEL_3, motorOutput.motorFrontRight);
}

int main(void)
{
    halCommonInit();
    sensorSystemInit(onSensorSystemUpdate);
    delayInit(delayTimerEventHandler);
    batteryMonitorInit();
    protocolInit(false);
    sensorSystemCalibrate();
    delayMs(SENSOR_STARTUP_TIMEOUT);
    sensorSystemGetCurrentOrientation(&orientationEstimate);
    flightControllerInit(SENSOR_SYSTEM_UPDATE_RATE, PWM_MAX);
    rcProcessorInit(&orientationEstimate, PWM_MAX);
    sensorSystemStartUpdateEvent();

    for (;;) {
        protocolProcess();
        if (protocolIsPacketReady()) {
            joysticksStatePacket = protocolGetJoystickValues();
            rcAliveTimer = 0;
        }

        if (isTimerEvent) {
            batteryMonitorProcess();
            uint8_t soc = batteryMonitorGetSoc();
            protocolSendBatterySoc(soc);
            isTimerEvent = false;
        }
    }
}

