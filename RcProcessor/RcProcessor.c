#include "RcProcessor.h"

#if 0

void getRotatedSetPoint(float ax, float ay, float *abx, float *aby)
{
    #define SIN_45 0.70710678118654752440084436210485f
    #define M_PI_F 3.14159265358979323846264338327950f
    #define DEG180 180.0f
    ax = ax * M_PI_F / DEG180;
    ay = ay * M_PI_F / DEG180;

    *aby = atanf(SIN_45 * (tanf(ay) - tanf(ax)));
    *abx = atanf(SIN_45 * (tanf(ax) + tanf(ay)));

    *aby = *aby * DEG180 / M_PI_F;
    *abx = *abx * DEG180 / M_PI_F;
    #undef M_PI_F
}

static float throttle, yaw, pitch, roll;
static void getSetPoint(FlightControllerSetPoint *setPoint)
{
    #define MAX_JOY_VALUE           2048
    #define MAX_THROTTLE            100.0f
    #define CLAMP_JOY(x)            (((x) < -MAX_JOY_VALUE) ? -MAX_JOY_VALUE : (((x) > MAX_JOY_VALUE) ? MAX_JOY_VALUE : (x)))
    #define MAX_QUAD_ANGLE          20.0f
    #define MAX_QUAD_RATE           0.5f
    #define MAX_QUAD_POWER_RATE     0.1f
    #define J1_CAL                  2104
    #define J2_CAL                  2073
    #define J3_CAL                  2025
    #define J4_CAL                  2008
    #define J5_CAL                  2064
    #define J6_CAL                  2105
    #define J7_CAL                  1994
    #define J8_CAL                  2010

    if (!protocolIsPacketReady() || !isPowerEnabled) {
        setPoint->throttle = 0;
        setPoint->pitch = 0;
        setPoint->roll = 0;
        setPoint->yaw = flightControllerGetYaw();
        return;
    }


    JoyValuesStatePacket *joystickValues = protocolGetJoystickValues();

    int32_t j1x = J1_CAL - (joystickValues->joyUpLeftXaxisStr & 0x0FFF);
    int32_t j1y = J2_CAL - (joystickValues->joyUpLeftYaxisB0 & 0x0FFF);
    int32_t j2x = J7_CAL - (joystickValues->joyUpRightXaxisB3 & 0x0FFF);
    int32_t j2y = J8_CAL - (joystickValues->joyUpRightYaxisStp & 0x0FFF);

    roll = (float) CLAMP_JOY(j2x) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;
    pitch = - (float) CLAMP_JOY(j2y) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;

    getRotatedSetPoint(roll, pitch, &roll, &pitch);



    float yawIncrement = - (float) CLAMP_JOY(j1x) * MAX_QUAD_RATE / (float) MAX_JOY_VALUE;

    if (yawIncrement < 0.001f && yawIncrement > -0.001f) {
        yawIncrement = 0;
    }

    if (fabsf(yaw - flightControllerGetYaw()) < 5.0f)
        yaw += yawIncrement;

    if (yaw > 180.0f) {
        yaw -= 360.0f;
    }

    if (yaw < -180.0f) {
        yaw += 360.0f;
    }

    float throttleIncrement = (float) CLAMP_JOY(j1y) * MAX_QUAD_POWER_RATE / (float) MAX_JOY_VALUE;


    if (throttleIncrement < 0.001f && throttleIncrement > -0.001f) {
        throttleIncrement = 0;
    }
    if (throttleIncrement < 0)
        throttleIncrement *= 2;
    if (!gpioHalGetBtConState()) {
        throttleIncrement = -0.01;
        pitch = 0;
        roll = 0;
    }

    throttle += throttleIncrement;

    if (throttle < 0.01f) {
        throttle = 0;
    }

    if (throttle > MAX_THROTTLE) {
        throttle = MAX_THROTTLE;
    }

    if (throttle < 0.001f && throttle > -0.001f) {
        yaw = flightControllerGetYaw();
        pitch = 0;
        roll = 0;
    }

    setPoint->pitch = pitch;
    setPoint->roll = roll;
    setPoint->yaw = yaw;
    setPoint->throttle = throttle;
}

#endif
