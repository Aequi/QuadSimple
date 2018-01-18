#ifndef __ADC_HAL_H__
#define __ADC_HAL_H__

#include <stdbool.h>
#include <stdint.h>

#define ADC_MAX_VAL     4095
#define ADC_REF_VAL_MV  3300

void adcHalInit(void);
uint32_t adcHalGetValue(void);

#endif
