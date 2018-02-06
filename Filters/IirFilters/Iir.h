#ifndef __FILTER_H__
#define __FILTER_H__
#include <stdint.h>

typedef struct lpfTwoPoleIirContext {
    float prevValue0;
    float prevValue1;

    float a0;
    float a1;
    float a2;

    float b1;
    float b2;
} lpfTwoPoleIirContext;

int32_t fltSinglePoleIirDecayFromFc(int32_t cutOffFrequency, int32_t samplingFrequency);
int16_t lpfSinglePoleIirApply(int32_t *filterContext, int32_t inputValue, int32_t decayValue);
float hpfSinglePoleIirApply(float *filterContext, float inputValue, float decayValue);

void lpfTwoPoleIirInit(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float cutOffFrequency, float samplingFrequency);
float lpfTwoPoleIirReset(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float inputValue);
float lpfTwoPoleIirApply(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float inputValue);


#endif //__FILTER_H__
