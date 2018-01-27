#ifndef __BATTERY_MONITOR_H__
#define __BATTERY_MONITOR_H__

#include "stdint.h"

void batteryMonitorInit(void);
void batteryMonitorProcess(void);
uint8_t batteryMonitorGetSoc(void);

#endif /* __BATTERY_MONITOR_H__ */
