#ifndef __FLIGHT_CONTROLLER_H__
#define __FLIGHT_CONTROLLER_H__

#include "stdint.h"
#include "stdbool.h"

typedef struct Quaternion {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct FlightControllerSetPoint {
    float throttle;
    float pitch;
    float roll;
    float yaw;
} FlightControllerSetPoint;

typedef struct FlightControllerOrientationEstimate {
    Quaternion orientation;

    float x;
    float y;
    float z;

    float vx;
    float vy;
    float vz;
} FlightControllerOrientationEstimate;

typedef struct FlightControllerMotorOutput {
    int32_t motorFrontLeft;
    int32_t motorBackLeft;
    int32_t motorBackRight;
    int32_t motorFrontRight;
} FlightControllerMotorOutput;

void flightControllerInit(uint32_t updateRate);

void flightControllerUpdate(FlightControllerSetPoint *setPoint,
                            FlightControllerOrientationEstimate *orientationEstimate,
                            FlightControllerMotorOutput *motorOutput);

float flightControllerGetYaw(void);
#endif
