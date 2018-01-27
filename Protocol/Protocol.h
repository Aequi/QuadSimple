#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)
typedef struct ProtocolJoystickPacket {
    uint16_t joyLeftX;
    uint16_t joyLeftY;
    uint16_t joyRightX;
    uint16_t joyRightY;
    uint8_t buttonCode;
} ProtocolJoystickPacket;
#pragma pack(pop)

void protocolProcess(void);
ProtocolJoystickPacket protocolGetJoystickValues(void);
bool protocolIsPacketReady(void);
void protocolSendBatterySoc(uint8_t soc);

#endif
