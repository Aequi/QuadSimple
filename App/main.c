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
#define LOW_BAT_TIMEOUT             2000
#define LOW_BAT_THRESHOLD           10

static bool isTimerEvent = false;
static uint32_t rcAliveTimer = 0;
static uint32_t lowBatCounter = 0;

void delayTimerEventHandler(void)
{
    if (rcAliveTimer++ >= RC_ALIVE_TIMEOUT) {
        rcAliveTimer = RC_ALIVE_TIMEOUT;
    }

    if (lowBatCounter++ >= LOW_BAT_TIMEOUT) {
        lowBatCounter = LOW_BAT_TIMEOUT;
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

    if (rcAliveTimer >= RC_ALIVE_TIMEOUT || lowBatCounter >= LOW_BAT_TIMEOUT) {
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
    delayInit(delayTimerEventHandler);
    batteryMonitorInit();
    protocolInit(false);
    sensorSystemInit(onSensorSystemUpdate, SENSOR_SYSTEM_UPDATE_RATE);
    sensorSystemCalibrate(false);
    delayMs(SENSOR_STARTUP_TIMEOUT);
    sensorSystemGetCurrentOrientation(&orientationEstimate);
    flightControllerInit(SENSOR_SYSTEM_UPDATE_RATE, PWM_MAX);
    rcProcessorInit(&orientationEstimate, PWM_MAX);
    sensorSystemStartUpdateEvent(true);

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
            if (soc > LOW_BAT_THRESHOLD) {
                lowBatCounter = 0;
            }
            isTimerEvent = false;
        }
    }
}

