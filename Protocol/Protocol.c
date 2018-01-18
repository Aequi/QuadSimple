#include "Protocol.h"
#include "UartHal.h"


#define JOY_STOP_SEQUENCE               0xFFFF
#define JOY_PRE_STOP_ZERO_BITS_MASK     0xF000

JoyValuesStatePacket joysticksStatePacket;

typedef enum ProtocolState {
    PROTOCOL_STATE_START_BYTE,
    PROTOCOL_STATE_SECOND_START_BYTE,
    PROTOCOL_STATE_THIRD_START_BYTE,
    PROTOCOL_STATE_SYNCED,
} ProtocolState;

static volatile bool isBtRxReady = false;
static void protocolProcessByte(uint8_t dataByte)
{
    static uint32_t packetBuffer[(sizeof(JoyValuesStatePacket) + sizeof(uint32_t)) / sizeof(uint32_t)];
    static uint32_t byteCounter;
    uint8_t *pPacketBuffer = (uint8_t *) packetBuffer;
    JoyValuesStatePacket *pPacketBufferCasted = (JoyValuesStatePacket *) pPacketBuffer;

    static ProtocolState protocolState = PROTOCOL_STATE_START_BYTE;

    switch (protocolState) {
    case PROTOCOL_STATE_START_BYTE :
        if (!(dataByte & 0xF0)) {
            protocolState = PROTOCOL_STATE_SECOND_START_BYTE;
        }
        break;
    case PROTOCOL_STATE_SECOND_START_BYTE :
    case PROTOCOL_STATE_THIRD_START_BYTE :
        if (dataByte == 0xFF) {
            if (protocolState == PROTOCOL_STATE_SECOND_START_BYTE) {
                protocolState = PROTOCOL_STATE_THIRD_START_BYTE;
            } else {
                protocolState = PROTOCOL_STATE_SYNCED;
            }
        } else {
            protocolState = PROTOCOL_STATE_START_BYTE;
        }
        break;
    case PROTOCOL_STATE_SYNCED :
        pPacketBuffer[byteCounter++] = dataByte;

        if (byteCounter >= sizeof(JoyValuesStatePacket)) {
            byteCounter = 0;
            if (pPacketBufferCasted->joyStopSequence == JOY_STOP_SEQUENCE && !(pPacketBufferCasted->joyUpRightYaxisStp & JOY_PRE_STOP_ZERO_BITS_MASK)) {
                joysticksStatePacket = *pPacketBufferCasted;
                isBtRxReady = true;
            } else {
                protocolState = PROTOCOL_STATE_START_BYTE;
            }
        }
        break;
    }
}

void protocolProcessBuffer(void)
{
    uint32_t lastByte = uartBtGetCurrentRxBuffIdx();
    static uint32_t firstByte = 0;
    uint32_t byteCounter = firstByte;

    while (byteCounter != lastByte) {
        protocolProcessByte(uartCmdInBuffer[byteCounter]);
        byteCounter = (byteCounter + 1) & UART_CMD_BUFF_MASK;
    }

    firstByte = lastByte;
}

JoyValuesStatePacket *protocolGetJoystickValues(void)
{
    return &joysticksStatePacket;
}

bool protocolIsPacketReady(void)
{
    return isBtRxReady;
}
