#include "FlightController.h"
#include "PidController.h"

#include <math.h>

#define CIRCLE_DEGREES  360.0f

#define PID_PITCH_KP    3.0f
#define PID_PITCH_KI    0.8f
#define PID_PITCH_KD    1.2f

#define PID_PITCH_POSITIVE_INTEGRAL_SATURATION_VALUE    20.0f
#define PID_PITCH_NEGATIVE_INTEGRAL_SATURATION_VALUE    -20.0f


#define PID_ROLL_KP     3.0f
#define PID_ROLL_KI     0.8f
#define PID_ROLL_KD     1.2f

#define PID_ROLL_POSITIVE_INTEGRAL_SATURATION_VALUE     20.0f
#define PID_ROLL_NEGATIVE_INTEGRAL_SATURATION_VALUE    -20.0f


#define PID_YAW_KP      4.0f
#define PID_YAW_KI      1.0f
#define PID_YAW_KD      0.7f

#define PID_YAW_POSITIVE_INTEGRAL_SATURATION_VALUE      36.0f
#define PID_YAW_NEGATIVE_INTEGRAL_SATURATION_VALUE     -36.0f


static PidControllerF pidControllerPitch, pidControllerRoll, pidControllerYaw;

static float controllResolution;

void flightControllerInit(uint32_t updateRate, uint32_t controlResolution)
{
    float res = controlResolution;
    controllResolution = res;
    pidControllerInitializeF(&pidControllerPitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, res, -res,
                             PID_PITCH_POSITIVE_INTEGRAL_SATURATION_VALUE, PID_PITCH_NEGATIVE_INTEGRAL_SATURATION_VALUE,
                             0.0f, false, 1.0f / (float) updateRate);

    pidControllerInitializeF(&pidControllerRoll, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, res, -res,
                             PID_ROLL_POSITIVE_INTEGRAL_SATURATION_VALUE, PID_ROLL_NEGATIVE_INTEGRAL_SATURATION_VALUE,
                             0.0f, false, 1.0f / (float) updateRate);

    pidControllerInitializeF(&pidControllerYaw, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, res, -res,
                             PID_YAW_POSITIVE_INTEGRAL_SATURATION_VALUE, PID_YAW_NEGATIVE_INTEGRAL_SATURATION_VALUE,
                             0.0f, false, 1.0f / (float) updateRate);
}

static inline uint32_t flightControllerClampPower(float power)
{
    return (((power) < 0.0) ? 0.0 : (((power) > controllResolution) ? controllResolution : (power)));
}

void flightControllerUpdate(FlightControllerSetPoint *setPoint,
                            FlightControllerOrientationEstimate *orientationEstimate,
                            FlightControllerMotorOutput *motorOutput)
{
    if (setPoint->throttle <= 1.0f) {
        motorOutput->motorFrontLeft = 0.0f;
        motorOutput->motorBackLeft = 0.0f;
        motorOutput->motorBackRight = 0.0f;
        motorOutput->motorFrontRight = 0.0f;
    } else {
		float pitchPidOut, rollPidOut, yawPidOut;
        pitchPidOut = pidControllerUpdateF(&pidControllerPitch, orientationEstimate->currentPitch - setPoint->pitch);
        rollPidOut = pidControllerUpdateF(&pidControllerRoll, orientationEstimate->currentRoll - setPoint->roll);

        float yawError;
        if ((orientationEstimate->currentYaw * setPoint->yaw < 0.0f) && (fabsf(orientationEstimate->currentYaw - setPoint->yaw) > CIRCLE_DEGREES / 2.0f)) {
            if (setPoint->yaw < 0.0f) {
                yawError = orientationEstimate->currentYaw - setPoint->yaw - CIRCLE_DEGREES;
            } else {
                yawError = orientationEstimate->currentYaw - setPoint->yaw + CIRCLE_DEGREES;
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
