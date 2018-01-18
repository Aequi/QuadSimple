#include "PidController.h"

#include <math.h>

void pidControllerReset(PidController *pidController)
{
    if (!pidController)
		return;

    pidController->integralComponent = 0;
    pidController->previousError = 0;
}

void pidControllerInitialize(PidController *pidController, int32_t kP, int32_t tI, int32_t tD,
                             int32_t positiveOutputClampingValue, int32_t negativeOutputClampingValue)
{
    if (!pidController)
		return;

    pidController->kP = kP;
    pidController->tI = tI;
    pidController->tD = tD;
    pidController->positiveOutputClampingValue = positiveOutputClampingValue;
    pidController->negativeOutputClampingValue = negativeOutputClampingValue;
    pidController->positiveIntegralClampingValue = positiveOutputClampingValue * tI / kP;
    pidController->negativeIntegralClampingValue = negativeOutputClampingValue * tI / kP;
    pidControllerReset(pidController);
}

static void pidClampIntegralValueAutomatic(const PidController *pidController, int32_t *integralComponent)
{
    if ((*integralComponent * pidController->kP / pidController->tI) > pidController->positiveOutputClampingValue)
        *integralComponent = pidController->positiveIntegralClampingValue;

    if ((*integralComponent * pidController->kP / pidController->tI) < pidController->negativeOutputClampingValue)
        *integralComponent = pidController->negativeIntegralClampingValue;
}

static void pidClampIntegralValue(const PidController *pidController, int32_t *integralComponent)
{
    if (*integralComponent > pidController->positiveIntegralClampingValue)
        *integralComponent = pidController->positiveIntegralClampingValue;

    if (*integralComponent < pidController->negativeIntegralClampingValue)
        *integralComponent = pidController->negativeIntegralClampingValue;
}

static void pidClampOutputValue(const PidController *pidController, int32_t *pidOutput)
{
    if (*pidOutput > pidController->positiveOutputClampingValue)
        *pidOutput = pidController->positiveOutputClampingValue;

    if (*pidOutput < pidController->negativeOutputClampingValue)
        *pidOutput = pidController->negativeOutputClampingValue;
}

int32_t pidControllerUpdate(PidController *pidController, int32_t error)
{
    #define AUTOMATIC   true
    if (!pidController)
		return 0;

    int32_t differentialComponent = pidController->tD * (error - pidController->previousError);
    pidController->integralComponent += ((error + pidController->previousError) >> 1);
    if (AUTOMATIC)
        pidClampIntegralValueAutomatic(pidController, &pidController->integralComponent);
    else {
        pidClampIntegralValue(pidController, &pidController->integralComponent);
    }
    pidController->integralComponent = pidController->integralComponent ? pidController->integralComponent / pidController->tI : 0;

    int32_t outputValue = (differentialComponent + pidController->integralComponent + error) * pidController->kP;

    pidController->previousError = error;

    pidClampOutputValue(pidController, &outputValue);
    return outputValue;
}

void pidControllerResetF(PidControllerF *pidController)
{
    if (!pidController)
		return;

    pidController->previousError = 0;
    pidController->integralComponent = 0;
}

void pidControllerInitializeF(PidControllerF *pidController, const float kP, const float kI, const float kD,
                             const float positiveOutputClampingValue, const float negativeOutputClampingValue,
                             const float positiveIntegralClampingValue, const float negativeIntegralClampingValue,
                             const float dComponentCutOffFrequency, const bool isFilterEnabled, const float dT)
{
    pidController->kP = kP;
    pidController->kI = kI;
    pidController->kD = kD;
    pidController->dT = dT;
    if ((!positiveIntegralClampingValue) && (!negativeIntegralClampingValue)) {
        pidController->positiveIntegralClampingValue = positiveOutputClampingValue / pidController->kI;
        pidController->negativeIntegralClampingValue = negativeOutputClampingValue / pidController->kI;
    } else {
        pidController->positiveIntegralClampingValue = positiveIntegralClampingValue;
        pidController->negativeIntegralClampingValue = negativeIntegralClampingValue;
    }
    pidController->positiveOutputClampingValue = positiveOutputClampingValue;
    pidController->negativeOutputClampingValue = negativeOutputClampingValue;

    if (isFilterEnabled) {
        lpfTwoPoleIirInit(&pidController->dComponentFilter, dComponentCutOffFrequency, 1.0f / dT);
    }


    pidControllerResetF(pidController);
}

static void pidClampIntegralValueAutomaticF(const PidControllerF *pidController, float *integralComponent)
{
    if ((*integralComponent * pidController->kI) > pidController->positiveOutputClampingValue)
        *integralComponent = pidController->positiveIntegralClampingValue;

    if ((*integralComponent * pidController->kI) < pidController->negativeOutputClampingValue)
        *integralComponent = pidController->negativeIntegralClampingValue;
}

static void pidClampIntegralValueF(const PidControllerF *pidController, float *integralComponent)
{
    if (*integralComponent > pidController->positiveIntegralClampingValue)
        *integralComponent = pidController->positiveIntegralClampingValue;

    if (*integralComponent < pidController->negativeIntegralClampingValue)
        *integralComponent = pidController->negativeIntegralClampingValue;
}

static void pidClampOutputValueF(const PidControllerF *pidController, float *pidOutput)
{
    if (*pidOutput > pidController->positiveOutputClampingValue)
        *pidOutput = pidController->positiveOutputClampingValue;

    if (*pidOutput < pidController->negativeOutputClampingValue)
        *pidOutput = pidController->negativeOutputClampingValue;
}

float pidControllerUpdateF(PidControllerF *pidController, const float error)
{
    float outputValue = pidController->kP * error;

    float derivative = (error - pidController->previousError) / pidController->dT;

    if (pidController->isFilterEnabled) {
        derivative = lpfTwoPoleIirApply(&pidController->dComponentFilter, derivative);
    }

    if (isnan(derivative) || isinf(derivative))
    {
        derivative = 0;
    }

    pidController->integralComponent += (error + pidController->previousError) * 0.5f * pidController->dT;

    if ((!pidController->positiveIntegralClampingValue) && (!pidController->negativeIntegralClampingValue)) {
        pidClampIntegralValueAutomaticF(pidController, &pidController->integralComponent);
    } else {
        pidClampIntegralValueF(pidController, &pidController->integralComponent);
    }

    outputValue += pidController->integralComponent * pidController->kI + derivative * pidController->kD;;

    pidController->previousError = error;

    pidClampOutputValueF(pidController, &outputValue);

    return outputValue;
}
