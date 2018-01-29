#include "FlightController.h"
#include "PidController.h"

#include <math.h>

static PidControllerF pidControllerPitch, pidControllerRoll, pidControllerYaw;

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

static inline uint32_t flightControllerClampPower(float power)
{
    return (((power) < 0.0) ? 0.0 : (((power) > controllResolution) ? controllResolution : (power)));
}

void flightControllerUpdate(FlightControllerSetPoint *setPoint,
                            FlightControllerOrientationEstimate *orientationEstimate,
                            FlightControllerMotorOutput *motorOutput)
{
    float pitchPidOut, rollPidOut, yawPidOut;

    if (setPoint->throttle <= 1.0f) {
        motorOutput->motorFrontLeft = 0;
        motorOutput->motorBackLeft = 0;
        motorOutput->motorBackRight = 0;
        motorOutput->motorFrontRight = 0;
    } else {
        pitchPidOut = pidControllerUpdateF(&pidControllerPitch, orientationEstimate->currentPitch - setPoint->pitch);
        rollPidOut = pidControllerUpdateF(&pidControllerRoll, orientationEstimate->currentRoll - setPoint->roll);

        float yawError;
        if ((orientationEstimate->currentYaw * setPoint->yaw < 0.0f) && (fabsf(orientationEstimate->currentYaw - setPoint->yaw) > 180.0f)) {
            if (setPoint->yaw < 0.0f) {
                yawError = orientationEstimate->currentYaw - setPoint->yaw - 360.0f;
            } else {
                yawError = orientationEstimate->currentYaw - setPoint->yaw + 360.0f;
            }
        } else {
            yawError = orientationEstimate->currentYaw - setPoint->yaw;
        }
        yawPidOut = pidControllerUpdateF(&pidControllerYaw, yawError);

        motorOutput->motorFrontLeft = flightControllerClampPower(setPoint->throttle - rollPidOut - yawPidOut);
        motorOutput->motorBackLeft = flightControllerClampPower(setPoint->throttle + pitchPidOut + yawPidOut);
        motorOutput->motorBackRight = flightControllerClampPower(setPoint->throttle + rollPidOut - yawPidOut);
        motorOutput->motorFrontRight = flightControllerClampPower(setPoint->throttle - pitchPidOut + yawPidOut);
    }
}
