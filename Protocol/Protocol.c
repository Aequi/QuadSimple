#include "Protocol.h"
#include "HalUart.h"
#include "HalGpio.h"
#include "Delay.h"

#define START_BIT   0x80

ProtocolJoystickPacket joysticksStatePacket;

static volatile bool isRfRxReady = false;
static volatile bool isRfTxReady = false;

void rfTxReady()
{
    isRfTxReady = true;
}

uint8_t crcUpdate(uint8_t crc, uint8_t byte)
{
    uint8_t val = (crc << 1) ^ byte, crcP = 0x89, j = 0;

    uint8_t x = (val & 0x80) ? val ^ crcP : val;
    for (j = 0; j < 7; ++j) {
        x <<= 1;
        if (x & 0x80) {
            x ^= crcP;
        }
    }

    return x;
}

uint8_t crc7(uint8_t buffer[], uint32_t length)
{
    uint8_t crc7 = 0, bcnt = 0;
    for (bcnt = 0; bcnt < length; bcnt++) {
        crc7 = crcUpdate(crc7, buffer[bcnt]);
    }

    return crc7;
}

void protocolUnpack(const uint8_t data[9])
{
    joysticksStatePacket.joyLeftX;
}

static void protocolProcessByte(uint8_t dataByte)
{
    static uint8_t packet[9];
    static bool isStartFound = false;
    static uint32_t byteCounter = 0;

    if (dataByte & START_BIT) {
        byteCounter = 0;
        isStartFound = true;
        packet[byteCounter++] = dataByte;
    } else if (isStartFound) {
        packet[byteCounter++] = dataByte;
        if (byteCounter < 9) {
            return;
        }

        if (crc7(packet, sizeof(packet)) == 0) {
            protocolUnpack(packet);
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
