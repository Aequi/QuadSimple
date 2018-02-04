#ifndef __SENSOR_SYSTEM_H__
#define __SENSOR_SYSTEM_H__

#include <stdbool.h>
#include <stdint.h>

#include "FlightController.h"

#define SENSOR_SYSTEM_UPDATE_RATE   250

typedef void (*SensorSystemUpdateCallback)(void);

void sensorSystemInit(SensorSystemUpdateCallback sensorSystemUpdateCb, uint32_t updateRate);
void sensorSystemCalibrate(bool isBiasUpdateNeeded);
void sensorSystemGetCurrentOrientation(FlightControllerOrientationEstimate *flightControllerOrientationEstimate);
void sensorSystemStartUpdateEvent(bool isUpdateEnabled);

#endif
