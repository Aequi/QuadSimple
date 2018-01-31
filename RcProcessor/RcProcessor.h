#ifndef __RC_PROCESSOR_H__
#define __RC_PROCESSOR_H__

#include <stdint.h>
#include <stdbool.h>

#include "FlightController.h"
#include "Protocol.h"

void rcProcessorGetSetPoint(FlightControllerSetPoint *setPoint, ProtocolJoystickPacket *joystickPacket, FlightControllerOrientationEstimate *flightControllerOrientatonEstimate, float maxOutputValue);

#endif
