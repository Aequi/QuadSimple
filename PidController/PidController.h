#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>

#include <Lpf.h>

typedef struct PidController {
    int32_t kP;
    int32_t tI;
    int32_t tD;

    int32_t integralComponent;
    int32_t previousError;

    int32_t positiveOutputClampingValue;
    int32_t negativeOutputClampingValue;
    int32_t positiveIntegralClampingValue;
    int32_t negativeIntegralClampingValue;
} PidController;

typedef struct PidControllerF {
    float kP;
    float kI;
    float kD;

    float integralComponent;
    float previousError;

    float positiveOutputClampingValue;
    float negativeOutputClampingValue;
    float positiveIntegralClampingValue;
    float negativeIntegralClampingValue;

    float dT;

    lpfTwoPoleIirContext dComponentFilter;
    bool isFilterEnabled;
} PidControllerF;

void pidControllerInitialize(PidController *pidController, const int32_t kP, const int32_t tI, const int32_t tD,
                             const int32_t positiveOutputClampingValue, const int32_t negativeOutputClampingValue);

int32_t pidControllerUpdate(PidController *pidController, const int32_t error);

void pidControllerReset(PidController *pidController);

void pidControllerInitializeF(PidControllerF *pidController, const float kP, const float kI, const float kD,
                              const float positiveOutputClampingValue, const float negativeOutputClampingValue,
                              const float positiveIntegralClampingValue, const float negativeIntegralClampingValue,
                              const float dComponentCutOffFrequency, bool isFilterEnabled, const float dT);

float pidControllerUpdateF(PidControllerF *pidController, const float error);

void pidControllerResetF(PidControllerF *pidController);

#endif
