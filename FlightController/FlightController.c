#include "FlightController.h"
#include "PidController.h"

#include <math.h>

#define M_PI_F  3.1415926535897932384626433832795f
static PidControllerF pidControllerPitch, pidControllerRoll, pidControllerYaw;
static float pitchPidOut, rollPidOut, yawPidOut;
static float currentPitch, currentRoll, currentYaw;
static float controllResolution;

void flightControllerInit(uint32_t updateRate, uint32_t controlResolution)
{
    float res = controlResolution;
    controllResolution = res;
    pidControllerInitializeF(&pidControllerPitch, 7.5f, 6.2f, 1.4f, res, -res,
                             20.0f, -20.0f, 0.0f, false, 1.0f / (float) updateRate);
    pidControllerInitializeF(&pidControllerRoll, 7.5f, 6.2f, 1.4f, res, -res,
                             20.0f, -20.0f, 0.0f, false, 1.0f / (float) updateRate);
    pidControllerInitializeF(&pidControllerYaw, 4.0f, 1.0f, 0.5f, res, -res,
                             36.0f, -36.0f, 0.0f, false, 1.0f / (float) updateRate);
}

static void eulerFromQuaternion(Quaternion *quaternion)
{
    float gx = 2.0f * (quaternion->x * quaternion->z - quaternion->w * quaternion->y);
    float gy = 2.0f * (quaternion->w * quaternion->x + quaternion->y * quaternion->z);
    float gz = quaternion->w * quaternion->w - quaternion->x * quaternion->x - quaternion->y * quaternion->y + quaternion->z * quaternion->z;

    if (gx > 1.0f) {
        gx = 1.0f;
    }
    if (gx < -1.0f) {
        gx = -1.0f;
    }

  currentYaw = atan2f(2.0f * (quaternion->w * quaternion->z + quaternion->x * quaternion->y),
                quaternion->w * quaternion->w + quaternion->x * quaternion->x - quaternion->y * quaternion->y - quaternion->z * quaternion->z) * 180.0f / M_PI_F;
  currentPitch = asinf(gx) * 180.0f / M_PI_F;
  currentRoll = atan2f(gy, gz) * 180.0f / M_PI_F;
}

void flightControllerUpdate(FlightControllerSetPoint *setPoint,
                            FlightControllerOrientationEstimate *orientationEstimate,
                            FlightControllerMotorOutput *motorOutput)
{
    #define CLAMP_POWER(x) (((x) < 0.0) ? 0.0 : (((x) > controllResolution) ? controllResolution : (x)))

    eulerFromQuaternion(&orientationEstimate->orientation);
    if (setPoint->throttle <= 1.0f) {
        motorOutput->motorFrontLeft = 0;
        motorOutput->motorBackLeft = 0;
        motorOutput->motorBackRight = 0;
        motorOutput->motorFrontRight = 0;
    } else {
        pitchPidOut = pidControllerUpdateF(&pidControllerPitch, currentPitch - setPoint->pitch);
        rollPidOut = pidControllerUpdateF(&pidControllerRoll, currentRoll - setPoint->roll);

        float yawError;
        if ((currentYaw * setPoint->yaw < 0.0f) && (fabsf(currentYaw - setPoint->yaw) > 180.0f)) {
            if (setPoint->yaw < 0.0f) {
                yawError = currentYaw - setPoint->yaw - 360.0f;
            } else {
                yawError = currentYaw - setPoint->yaw + 360.0f;
            }
        } else {
            yawError = currentYaw - setPoint->yaw;
        }
        yawPidOut = pidControllerUpdateF(&pidControllerYaw, yawError);

        motorOutput->motorFrontLeft = CLAMP_POWER(setPoint->throttle - rollPidOut - yawPidOut);
        motorOutput->motorBackLeft = CLAMP_POWER(setPoint->throttle + pitchPidOut + yawPidOut);
        motorOutput->motorBackRight = CLAMP_POWER(setPoint->throttle + rollPidOut - yawPidOut);
        motorOutput->motorFrontRight = CLAMP_POWER(setPoint->throttle - pitchPidOut + yawPidOut);
    }
}

float flightControllerGetYaw(void)
{
    return currentYaw;
}
