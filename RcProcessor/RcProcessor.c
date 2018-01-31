#include "RcProcessor.h"
#include <math.h>

#define DEG180 180.0f

static void getRotatedSetPoint(float ax, float ay, float *abx, float *aby)
{
    #define SIN_45 0.70710678118654752440084436210485f
    #define M_PI_F 3.14159265358979323846264338327950f

    ax = ax * M_PI_F / DEG180;
    ay = ay * M_PI_F / DEG180;

    *aby = atanf(SIN_45 * (tanf(ay) - tanf(ax)));
    *abx = atanf(SIN_45 * (tanf(ax) + tanf(ay)));

    *aby = *aby * DEG180 / M_PI_F;
    *abx = *abx * DEG180 / M_PI_F;
    #undef M_PI_F
    #undef SIN_45
}

void rcProcessorGetSetPoint(FlightControllerSetPoint *setPoint, ProtocolJoystickPacket *joystickPacket, FlightControllerOrientationEstimate *flightControllerOrientatonEstimate, float maxOutputValue)
{
    #define MAX_JOY_VALUE           2048
    #define JOY_VALUE_MASK          0x0FFF
    #define MAX_THROTTLE            maxOutputValue
    #define CLAMP_JOY(x)            (((x) < -MAX_JOY_VALUE) ? -MAX_JOY_VALUE : (((x) > MAX_JOY_VALUE) ? MAX_JOY_VALUE : (x)))
    #define MAX_QUAD_ANGLE          20.0f
    #define MAX_QUAD_RATE           0.5f
    #define MAX_QUAD_POWER_RATE     0.1f
    #define JOY_EPS                 0.001f
    #define MAX_YAW_ERROR           5.0f
    #define J1_CAL                  2104
    #define J2_CAL                  2073
    #define J3_CAL                  2025
    #define J4_CAL                  2008

    static float throttle = 0, yaw = 0, pitch = 0, roll = 0;

    int32_t j1x = J1_CAL - (joystickPacket->leftX & JOY_VALUE_MASK);
    int32_t j1y = J2_CAL - (joystickPacket->leftY & JOY_VALUE_MASK);
    int32_t j2x = J3_CAL - (joystickPacket->rightX & JOY_VALUE_MASK);
    int32_t j2y = J4_CAL - (joystickPacket->rightY & JOY_VALUE_MASK);

    roll = (float) CLAMP_JOY(j2x) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;
    pitch = - (float) CLAMP_JOY(j2y) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;

    getRotatedSetPoint(roll, pitch, &roll, &pitch);

    float yawIncrement = - (float) CLAMP_JOY(j1x) * MAX_QUAD_RATE / (float) MAX_JOY_VALUE;

    if (yawIncrement < JOY_EPS && yawIncrement > -JOY_EPS) {
        yawIncrement = 0;
    }

    if (fabsf(yaw - flightControllerOrientatonEstimate->currentYaw) < MAX_YAW_ERROR) {
        yaw += yawIncrement;
    }

    if (yaw > DEG180) {
        yaw -= DEG180 * 2;
    }

    if (yaw < -DEG180) {
        yaw += DEG180 * 2;
    }

    float throttleIncrement = (float) CLAMP_JOY(j1y) * MAX_QUAD_POWER_RATE / (float) MAX_JOY_VALUE;

    if (throttleIncrement < JOY_EPS && throttleIncrement > -JOY_EPS) {
        throttleIncrement = 0;
    }

    if (throttleIncrement < 0) {
        throttleIncrement *= 2;
    }

    throttle += throttleIncrement;

    if (throttle < JOY_EPS) {
        throttle = 0;
    }

    if (throttle > MAX_THROTTLE) {
        throttle = MAX_THROTTLE;
    }

    if (throttle < JOY_EPS && throttle > -JOY_EPS) {
        yaw = flightControllerOrientatonEstimate->currentYaw;
        pitch = 0;
        roll = 0;
    }

    setPoint->pitch = pitch;
    setPoint->roll = roll;
    setPoint->yaw = yaw;
    setPoint->throttle = throttle;

    #undef MAX_JOY_VALUE
    #undef JOY_VALUE_MASK
    #undef MAX_THROTTLE
    #undef CLAMP_JOY
    #undef MAX_QUAD_ANGLE
    #undef MAX_QUAD_RATE
    #undef MAX_QUAD_POWER_RATE
    #undef JOY_EPS
    #undef MAX_YAW_ERROR
    #undef J1_CAL
    #undef J2_CAL
    #undef J3_CAL
    #undef J4_CAL
}
