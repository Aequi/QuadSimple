#include <math.h>

#include "iir.h"

#define B_256_SCALER    8
#define B_128_SCALER    7
#define E_F             2.7182818284590452353602874713527f
#define PI_F            3.1415926535897932384626433832795f

// f0 - 3 dB cutoff frequency
// fs = sample frequency
// xk = input value at index k
// yk = filter output value at index k
// g = 1
// p = √2
// f* = f0 ⁄ fs
// 0 < f* < 1⁄8
// (unstable when f* ≥ 1⁄4)
// ω0 = tan(π f*)
// K1 = p * ω0
// K2 = g * ω0 ^ 2
// A0 = K2 ⁄ (1 + K1 + K2)
// A1 = 2 * A0
// A2 = A0
// B1 = 2 * A0 * (1 ⁄ K2 − 1)
// B2 = 1 − (A0 + A1 + A2 + B1)
void lpfTwoPoleIirInit(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float cutOffFrequency, float samplingFrequency)
{
    #define BUTTER_G    1.0f
    #define BUTTER_P    1.4142135623730950488016887242097f

    if (!lpfTwoPoleIirContext) {
        return;
    }

    float f0ToFs = cutOffFrequency / samplingFrequency;
    float omega = tanf(PI_F * f0ToFs);
    float K1 = BUTTER_P * omega;
    float K2 = BUTTER_G * omega * omega;

    lpfTwoPoleIirContext->a0 = K2 / (1.0f + K1 + K2);
    lpfTwoPoleIirContext->a1 = 2.0f * lpfTwoPoleIirContext->a0;
    lpfTwoPoleIirContext->a2 = lpfTwoPoleIirContext->a0;
    lpfTwoPoleIirContext->b1 = 2.0f * lpfTwoPoleIirContext->a0 * (1.0f / K2 - 1.0f);
    lpfTwoPoleIirContext->b2 = 1.0f - (lpfTwoPoleIirContext->a0 +  lpfTwoPoleIirContext->a1 + lpfTwoPoleIirContext->a2 + lpfTwoPoleIirContext->b1 + lpfTwoPoleIirContext->b2);

    lpfTwoPoleIirContext->prevValue0 = 0.0f;
    lpfTwoPoleIirContext->prevValue1 = 0.0f;
}

float lpfTwoPoleIirReset(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float inputValue)
{
  lpfTwoPoleIirContext->prevValue0 = inputValue / (lpfTwoPoleIirContext->a0 + lpfTwoPoleIirContext->a1 + lpfTwoPoleIirContext->a2);
  lpfTwoPoleIirContext->prevValue1 = lpfTwoPoleIirContext->prevValue0;
  return lpfTwoPoleIirApply(lpfTwoPoleIirContext, inputValue);
}

float lpfTwoPoleIirApply(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float inputValue)
{
    float currentValue = inputValue - lpfTwoPoleIirContext->prevValue0 * lpfTwoPoleIirContext->b1 - lpfTwoPoleIirContext->prevValue1 * lpfTwoPoleIirContext->b2;

    currentValue = isfinite(currentValue) ? inputValue : currentValue;

    float outputValue = currentValue * lpfTwoPoleIirContext->a0 + lpfTwoPoleIirContext->prevValue0 * lpfTwoPoleIirContext->a1 + lpfTwoPoleIirContext->prevValue1 * lpfTwoPoleIirContext->a2;

    lpfTwoPoleIirContext->prevValue1 = lpfTwoPoleIirContext->prevValue0;
    lpfTwoPoleIirContext->prevValue0 = currentValue;

    return outputValue;
}

// d = e ^ (-2 * π * fc)
int32_t fltSinglePoleIirDecayFromFc(int32_t cutOffFrequency, int32_t samplingFrequency)
{
    float f0ToFs = (float) cutOffFrequency / (float) samplingFrequency;
    float d = powf(E_F, -2.0f * PI_F * f0ToFs);
    return d * 256.0f + 0.5f;
}

// y[n] = y[n - 1] * d + x[n] * (1 - d)
// y += (1 - d) * (x - y)
// d - decay, x -  input, y - output
int16_t lpfSinglePoleIirApply(int32_t *filterContext, int32_t inputValue, int32_t decayValue)
{
    if (filterContext == NULL) {
        return 0;
    }

    inputValue <<= B_256_SCALER;
    *filterContext += ((inputValue - *filterContext) >> B_256_SCALER) * ((1 << (B_256_SCALER)) - decayValue);
    return (*filterContext >> B_256_SCALER) + ((*filterContext & (1 << B_128_SCALER)) >> (B_128_SCALER));
}

float hpfSinglePoleIirApply(float *filterContext, float inputValue, float decayValue)
{
    if (filterContext == NULL) {
        return 0.0;
    }

    float f = inputValue + decayValue * (*filterContext), y = 0;

    y = f - (*filterContext);
    *filterContext = f;

    return y;
}
