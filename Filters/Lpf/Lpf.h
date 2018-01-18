#ifndef FILTER_H_
#define FILTER_H_
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

int32_t lpfSinglePoleIirDecayFromFc(int32_t cutOffFrequency, int32_t samplingFrequency);
int16_t lpfSinglePoleIirApply(int32_t *filterContext, int32_t inputValue, int32_t decayValue);

void lpfTwoPoleIirInit(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float cutOffFrequency, float samplingFrequency);
float lpfTwoPoleIirReset(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float inputValue);
float lpfTwoPoleIirApply(lpfTwoPoleIirContext *lpfTwoPoleIirContext, float inputValue);


#endif //FILTER_H_
