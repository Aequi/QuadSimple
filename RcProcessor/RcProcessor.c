#include "RcProcessor.h"
#include <math.h>

#define DEG180                  180.0f
#define MAX_JOY_VALUE           2048
#define JOY_VALUE_MASK          0x0FFF
#define MAX_THROTTLE            maxOutputValueStatic
#define CLAMP_JOY(x)            (((x) < -MAX_JOY_VALUE) ? -MAX_JOY_VALUE : (((x) > MAX_JOY_VALUE) ? MAX_JOY_VALUE : (x)))
#define MAX_QUAD_ANGLE          20.0f
#define MAX_QUAD_RATE           1.0f
#define MAX_QUAD_POWER_RATE     1.0f
#define JOY_EPS                 0.1f
#define MAX_YAW_ERROR           180.0f
#define J1_CAL                  1972
#define J2_CAL                  2098
#define J3_CAL                  2028
#define J4_CAL                  2029
#define SIN_45                  0.70710678118654752440084436210485f
#define M_PI_F                  3.14159265358979323846264338327950f


static float throttle = 0, yaw = 0, pitch = 0, roll = 0, maxOutputValueStatic = 0;

static void getRotatedSetPoint(float ax, float ay, float *abx, float *aby)
{
    ax = ax * M_PI_F / DEG180;
    ay = ay * M_PI_F / DEG180;

    *aby = atanf(SIN_45 * (tanf(ay) - tanf(ax)));
    *abx = atanf(SIN_45 * (tanf(ax) + tanf(ay)));

    *aby = *aby * DEG180 / M_PI_F;
    *abx = *abx * DEG180 / M_PI_F;
}

void rcProcessorInit(FlightControllerOrientationEstimate *flightControllerOrientatonEstimate, float maxOutputValue)
{
    maxOutputValueStatic = maxOutputValue;
    throttle = 0;
    pitch = 0;
    roll = 0;
    yaw = flightControllerOrientatonEstimate->currentYaw;
}

    static int32_t j1x;
    static int32_t j1y;
    static int32_t j2x;
    static int32_t j2y;

void rcProcessorGetSetPoint(FlightControllerSetPoint *setPoint, ProtocolJoystickPacket *joystickPacket, FlightControllerOrientationEstimate *flightControllerOrientatonEstimate)
{
    #define PRE_SHIFT_BIT_COUNT   8
    #define LOW_PASS_FILTER_KOEF  6
    #define LOW_PASS_FILTER_KOEF_MAX (1 << PRE_SHIFT_BIT_COUNT)

    j1x = ((j1x * (LOW_PASS_FILTER_KOEF_MAX - LOW_PASS_FILTER_KOEF)) + ((joystickPacket->leftX & JOY_VALUE_MASK) - J1_CAL) * LOW_PASS_FILTER_KOEF) >> PRE_SHIFT_BIT_COUNT;
    j1y = ((j1y * (LOW_PASS_FILTER_KOEF_MAX - LOW_PASS_FILTER_KOEF)) + ((joystickPacket->leftY & JOY_VALUE_MASK) - J2_CAL) * LOW_PASS_FILTER_KOEF) >> PRE_SHIFT_BIT_COUNT;
    j2x = ((j2x * (LOW_PASS_FILTER_KOEF_MAX - LOW_PASS_FILTER_KOEF)) + ((joystickPacket->rightX & JOY_VALUE_MASK) - J3_CAL) * LOW_PASS_FILTER_KOEF) >> PRE_SHIFT_BIT_COUNT;
    j2y = ((j2y * (LOW_PASS_FILTER_KOEF_MAX - LOW_PASS_FILTER_KOEF)) + ((joystickPacket->rightY & JOY_VALUE_MASK) - J4_CAL) * LOW_PASS_FILTER_KOEF) >> PRE_SHIFT_BIT_COUNT;

    roll = -(float) CLAMP_JOY(j2x) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;
    pitch = - (float) CLAMP_JOY(j2y) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;

    getRotatedSetPoint(roll, pitch, &roll, &pitch);

    float yawIncrement = - (float) CLAMP_JOY(j1x) * MAX_QUAD_RATE / (float) MAX_JOY_VALUE;

    if (yawIncrement < JOY_EPS && yawIncrement > -JOY_EPS) {
        yawIncrement = 0;
    }

    if (fabsf(yaw - flightControllerOrientatonEstimate->currentYaw) < MAX_YAW_ERROR) {
        yaw += yawIncrement;
    } else {
        yaw = flightControllerOrientatonEstimate->currentYaw;
    }

    if (yaw > DEG180) {
        yaw -= DEG180 * 2.0;
    }

    if (yaw < -DEG180) {
        yaw += DEG180 * 2.0;
    }

    float throttleIncrement = (float) CLAMP_JOY(j1y) * MAX_QUAD_POWER_RATE / (float) MAX_JOY_VALUE;

    if (throttleIncrement < 0) {
        throttleIncrement *= 4.0;
    }

    if (throttleIncrement < JOY_EPS && throttleIncrement > -JOY_EPS) {
        throttleIncrement = 0;
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
}

void rcProcessorGetIdleSetPoint(FlightControllerSetPoint *setPoint, FlightControllerOrientationEstimate *flightControllerOrientatonEstimate)
{
    throttle -= MAX_QUAD_POWER_RATE;

    if (throttle < JOY_EPS) {
        throttle = 0;
    }

    if (throttle > MAX_THROTTLE) {
        throttle = MAX_THROTTLE;
    }

    if (throttle < JOY_EPS && throttle > -JOY_EPS) {
        throttle = 0;
    }

    setPoint->throttle = throttle;
    setPoint->roll = roll = 0;
    setPoint->pitch = pitch = 0;
    setPoint->yaw = flightControllerOrientatonEstimate->currentYaw;
}
