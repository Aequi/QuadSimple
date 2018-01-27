#include "BatteryMonitor.h"
#include "HalAdc.h"
#include <stddef.h>

#define BATTERY_MAX_VOLTAGE_MV      4200
#define BATTERY_MIN_VOLTAGE_MV      3300
#define REF_VOLTAGE_MV              3300
#define BATTERY_DIVIDER_RATIO       2
#define ADC_RESOLUTION_BIT_SHIFT    12
#define MAX_PERCENT                 100

static uint8_t soc;
const uint16_t *adcData;
uint32_t adcDataLength = 0;

volatile bool adcValuesReady = false;

void adcDataReady(const uint16_t data[], uint32_t length)
{
    if (adcValuesReady == false) {
        adcData = data;
        adcDataLength = length;
        adcValuesReady = true;
        halAdcStartStop(false);
    }
}

void batteryMonitorInit(void)
{
    halAdcInit(adcDataReady);
    halAdcStartStop(true);
    adcValuesReady = false;
}

void batteryMonitorProcess(void)
{
    if (adcValuesReady == false) {
        return;
    }

    uint32_t valueCounter = 0;
    uint32_t adcValue = 0;
    for (valueCounter = 0; valueCounter < adcDataLength; valueCounter++) {
        adcValue += adcData[valueCounter];
    }
    adcValue /= adcDataLength;
    adcValue = (adcValue * (REF_VOLTAGE_MV * BATTERY_DIVIDER_RATIO)) >> ADC_RESOLUTION_BIT_SHIFT;
    adcValue = (adcValue - BATTERY_MIN_VOLTAGE_MV) * MAX_PERCENT / (BATTERY_MAX_VOLTAGE_MV - BATTERY_MIN_VOLTAGE_MV);
    soc = adcValue;
    halAdcStartStop(true);
    adcValuesReady = false;
}

uint8_t batteryMonitorGetSoc(void)
{
    return soc;
}
