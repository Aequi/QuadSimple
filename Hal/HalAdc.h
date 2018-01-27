#ifndef __HAL_ADC_H__
#define __HAL_ADC_H__

#include <stdbool.h>
#include <stdint.h>

#define ADC_BUFF_SIZE       64
#define ADC_CHANNEL_COUNT   1

typedef void (*HalAdcDataReady)(const uint16_t data[], uint32_t length);

void halAdcInit(HalAdcDataReady halAdcDataReadyCb);
void halAdcStartStop(bool isStart);

#endif
