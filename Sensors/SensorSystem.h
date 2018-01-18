#ifndef __SENSOR_SYSTEM_H__
#define __SENSOR_SYSTEM_H__

#include <stdbool.h>
#include <stdint.h>

#define SENSOR_SYSTEM_UPDATE_RATE   500

void sensorSystemInit(void);
void sensorSystemCalibrate(float magnetometerBias[3], float magnetometerSensitivity[3], bool isBiasUpdateNeeded);

#endif
