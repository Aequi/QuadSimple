#include "Protocol.h"
#include "HalUart.h"
#include "HalGpio.h"
#include "Delay.h"

#include <stddef.h>

#define PACKET_LENGTH   9
#define START_BIT       0x80

ProtocolJoystickPacket joysticksStatePacket;

static volatile bool isRfRxReady = false;
static volatile bool isRfTxReady = false;

void rfTxReady()
{
    isRfTxReady = true;
}

static uint8_t crcUpdate(uint8_t crc, uint8_t byte)
{
    const uint8_t poly = 0x07;
    crc ^= byte;
    for (uint32_t i = 0; i < 0x08; i++) {
        crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
    }

    return crc;
}

static uint8_t crc8(uint8_t buffer[], uint32_t length)
{
    uint8_t crc8 = 0, bcnt = 0;
    for (bcnt = 0; bcnt < length; bcnt++) {
        crc8 = crcUpdate(crc8, buffer[bcnt]);
    }

    return crc8;
}

static void protocolUnpack(const uint8_t data[PACKET_LENGTH], ProtocolJoystickPacket *joystickData)
{
    if (data == NULL || joystickData == NULL) {
        return;
    }

    joystickData->leftX = (((data[0] & 0x7F) | ((data[7] & 0x01) << 7)) << 4) | (((data[1] & 0x7F) | ((data[7] & 0x02) << 6)) >> 4);
    joystickData->leftY = ((data[1] & 0x0F) << 8) | ((data[2] & 0x7F) | ((data[7] & 0x04) << 5));
    joystickData->rightX = (((data[3] & 0x7F) | ((data[7] & 0x08) << 4)) << 4) | (((data[4] & 0x7F) | ((data[7] & 0x10) << 3)) >> 4);
    joystickData->rightY = ((data[4] & 0x0F) << 8) | ((data[5] & 0x7F) | ((data[7] & 0x20) << 2));
    joystickData->buttonCode = data[6];

}

static void protocolProcessByte(uint8_t dataByte)
{
    static uint8_t packet[PACKET_LENGTH];
    static bool isStartFound = false;
    static uint32_t byteCounter = 0;

    if (dataByte & START_BIT) {
        byteCounter = 0;
        isStartFound = true;
        packet[byteCounter++] = dataByte;
    } else if (isStartFound) {
        packet[byteCounter++] = dataByte;
        if (byteCounter < PACKET_LENGTH) {
            return;
        }

        packet[8] |= (packet[7] & 0x40) << 1;
        packet[7] &= (~0x40);
        if (crc8(packet, sizeof(packet)) == 0) {
            protocolUnpack(packet, &joysticksStatePacket);
            isRfRxReady = true;
        } else {
            byteCounter = 0;
            isStartFound = false;
        }
    }
}

void protocolInit(bool isParameterSetNeeded)
{
    halGpioInit();

    if (isParameterSetNeeded) {
        halGpioEnableRfAt(true);

        delayMs(200);
        uartRfInit(115200);
        uartRfDmaStartStopRx(true);

        // Set configuration
        isRfTxReady = false;
        uartRfDmaTx((uint8_t *) "AT+B115200", sizeof("AT+B115200") - 1, rfTxReady);
        while (!isRfTxReady);
        isRfTxReady = false;
        uartRfDmaTx((uint8_t *) "AT+P7", sizeof("AT+P7") - 1, rfTxReady);
        while (!isRfTxReady);
        halGpioEnableRfAt(false);
        delayMs(20);
    } else {
        uartRfInit(115200);
        uartRfDmaStartStopRx(true);
        isRfTxReady = true;



    }

    delayMs(20);

    uartRfDmaStartStopRx(true);
}

void protocolProcess(void)
{
    uint32_t lastByte = uartRfGetCurrentRxBuffIdx();
    static uint32_t firstByte = 0;
    uint32_t byteCounter = firstByte;

    while (byteCounter != lastByte) {
        protocolProcessByte(uartCmdInBuffer[byteCounter]);
        byteCounter = (byteCounter + 1) & UART_CMD_BUFF_MASK;
    }

    firstByte = lastByte;
}

ProtocolJoystickPacket protocolGetJoystickValues(void)
{
    isRfRxReady = false;
    return joysticksStatePacket;
}

bool protocolIsPacketReady(void)
{
    return isRfRxReady;
}

void protocolSendBatterySoc(uint8_t soc)
{
    if (isRfTxReady == true) {
        static uint8_t batteryCharge = 0;
        batteryCharge = soc;
        isRfTxReady = false;
        uartRfDmaTx(&batteryCharge, sizeof(batteryCharge), rfTxReady);
    }
}
