#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)
typedef struct JoyValuesStatePacket {

    uint16_t joyUpLeftXaxisStr;
    uint16_t joyUpLeftYaxisB0;
    uint16_t joyDnLeftXaxis;
    uint16_t joyDnLeftYaxisB1;
    uint16_t joyDnRightXaxisB2;
    uint16_t joyDnRightYaxis;
    uint16_t joyUpRightXaxisB3;
    uint16_t joyUpRightYaxisStp;
    uint16_t joyStopSequence;
} JoyValuesStatePacket;
#pragma pack(pop)

void protocolProcessBuffer(void);
JoyValuesStatePacket *protocolGetJoystickValues(void);
bool protocolIsPacketReady(void);

#endif
