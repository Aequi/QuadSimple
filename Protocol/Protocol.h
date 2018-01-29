#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)
typedef struct ProtocolJoystickPacket {
    uint16_t leftX;
    uint16_t leftY;
    uint16_t rightX;
    uint16_t rightY;
    uint8_t buttonCode;
} ProtocolJoystickPacket;
#pragma pack(pop)

void protocolInit(bool isParameterSetNeeded);
void protocolProcess(void);
ProtocolJoystickPacket protocolGetJoystickValues(void);
bool protocolIsPacketReady(void);
void protocolSendBatterySoc(uint8_t soc);

#endif
