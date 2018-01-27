/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "system_stm32f4xx.h"

#include "HalCommon.h"
#include "AdcHal.h"
#include "ButtonHal.h"
#include "GpioHal.h"
#include "UartHal.h"
#include "TimersHal.h"
#include "SpiHal.h"
#include "LedController.h"
#include "FlightController.h"
#include "Protocol.h"
#include "SensorSystem.h"
#include <math.h>

#include "OrientationFilter.h"

#define LI_ION_FULL_VOLTAGE     4200

static volatile bool isPowerEnabled = false;

static volatile bool isTimerEvent = false;
static void onTimerEvent(void)
{
    isTimerEvent = true;
}

static volatile bool isBtTxReady = false;
static void onBtTxReady(void)
{
    isBtTxReady = true;
}

void getRotatedSetPoint(float ax, float ay, float *abx, float *aby)
{
    #define SIN_45 0.70710678118654752440084436210485f
    #define M_PI_F 3.14159265358979323846264338327950f
    #define DEG180 180.0f
    ax = ax * M_PI_F / DEG180;
    ay = ay * M_PI_F / DEG180;

    *aby = atanf(SIN_45 * (tanf(ay) - tanf(ax)));
    *abx = atanf(SIN_45 * (tanf(ax) + tanf(ay)));

    *aby = *aby * DEG180 / M_PI_F;
    *abx = *abx * DEG180 / M_PI_F;
    #undef M_PI_F
}

static float throttle, yaw, pitch, roll;
static void getSetPoint(FlightControllerSetPoint *setPoint)
{
    #define MAX_JOY_VALUE           2048
    #define MAX_THROTTLE            100.0f
    #define CLAMP_JOY(x)            (((x) < -MAX_JOY_VALUE) ? -MAX_JOY_VALUE : (((x) > MAX_JOY_VALUE) ? MAX_JOY_VALUE : (x)))
    #define MAX_QUAD_ANGLE          20.0f
    #define MAX_QUAD_RATE           0.5f
    #define MAX_QUAD_POWER_RATE     0.1f
    #define J1_CAL                  2104
    #define J2_CAL                  2073
    #define J3_CAL                  2025
    #define J4_CAL                  2008
    #define J5_CAL                  2064
    #define J6_CAL                  2105
    #define J7_CAL                  1994
    #define J8_CAL                  2010

    if (!protocolIsPacketReady() || !isPowerEnabled) {
        setPoint->throttle = 0;
        setPoint->pitch = 0;
        setPoint->roll = 0;
        setPoint->yaw = flightControllerGetYaw();
        return;
    }


    JoyValuesStatePacket *joystickValues = protocolGetJoystickValues();

    int32_t j1x = J1_CAL - (joystickValues->joyUpLeftXaxisStr & 0x0FFF);
    int32_t j1y = J2_CAL - (joystickValues->joyUpLeftYaxisB0 & 0x0FFF);
    int32_t j2x = J7_CAL - (joystickValues->joyUpRightXaxisB3 & 0x0FFF);
    int32_t j2y = J8_CAL - (joystickValues->joyUpRightYaxisStp & 0x0FFF);

    roll = (float) CLAMP_JOY(j2x) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;
    pitch = - (float) CLAMP_JOY(j2y) * MAX_QUAD_ANGLE / (float) MAX_JOY_VALUE;

    getRotatedSetPoint(roll, pitch, &roll, &pitch);



    float yawIncrement = - (float) CLAMP_JOY(j1x) * MAX_QUAD_RATE / (float) MAX_JOY_VALUE;

    if (yawIncrement < 0.001f && yawIncrement > -0.001f) {
        yawIncrement = 0;
    }

    if (fabsf(yaw - flightControllerGetYaw()) < 5.0f)
        yaw += yawIncrement;

    if (yaw > 180.0f) {
        yaw -= 360.0f;
    }

    if (yaw < -180.0f) {
        yaw += 360.0f;
    }

    float throttleIncrement = (float) CLAMP_JOY(j1y) * MAX_QUAD_POWER_RATE / (float) MAX_JOY_VALUE;


    if (throttleIncrement < 0.001f && throttleIncrement > -0.001f) {
        throttleIncrement = 0;
    }
    if (throttleIncrement < 0)
        throttleIncrement *= 2;
    if (!gpioHalGetBtConState()) {
        throttleIncrement = -0.01;
        pitch = 0;
        roll = 0;
    }

    throttle += throttleIncrement;

    if (throttle < 0.01f) {
        throttle = 0;
    }

    if (throttle > MAX_THROTTLE) {
        throttle = MAX_THROTTLE;
    }

    if (throttle < 0.001f && throttle > -0.001f) {
        yaw = flightControllerGetYaw();
        pitch = 0;
        roll = 0;
    }

    setPoint->pitch = pitch;
    setPoint->roll = roll;
    setPoint->yaw = yaw;
    setPoint->throttle = throttle;
}



float magnetometerBias[3], magnetometerSensitivity[3];

static OrientationFilterCtx orientationFilterCtx;
static OfVector3 accMeasurements, gyroMeasurements, magMeasurements;
static FlightControllerMotorOutput motorOutput;
static FlightControllerSetPoint setPoint;
static FlightControllerOrientationEstimate orientationEstimate;
const float magResolution = 4912.0f / 32760.0f;
static int16_t imuData[9];
static volatile bool isImuFrameReady = false;
static void onImuFrameReady(void)
{
    extern float fMagCalibrationX;
    extern float fMagCalibrationY;
    extern float fMagCalibrationZ;

    #define M_PI_F  3.1415926535897932384626433832795f
    uint8_t *data = spiHalImuGetBuffer();

    uint8_t *temp = (uint8_t *) imuData;

    temp[0] = data[1]; temp[1] = data[0];
    temp[2] = data[3]; temp[3] = data[2];
    temp[4] = data[5]; temp[5] = data[4];

    temp[6] = data[9]; temp[7] = data[8];
    temp[8] = data[11]; temp[9] = data[10];
    temp[10] = data[13]; temp[11] = data[12];

    temp[12] = data[14]; temp[13] = data[15];
    temp[14] = data[16]; temp[15] = data[17];
    temp[16] = data[18]; temp[17] = data[19];

    accMeasurements.x = (float) imuData[0] * 8.0f / 32767.0f;
    accMeasurements.y = (float) imuData[1] * 8.0f / 32767.0f;
    accMeasurements.z = (float) imuData[2] * 8.0f / 32767.0f;
    gyroMeasurements.x = (float) imuData[3] * 2000.0f * M_PI_F / 32767.0f / 180.0f;
    gyroMeasurements.y = (float) imuData[4] * 2000.0f * M_PI_F / 32767.0f / 180.0f;
    gyroMeasurements.z = (float) imuData[5] * 2000.0f * M_PI_F / 32767.0f / 180.0f;

    magMeasurements.x = (float) imuData[7] * fMagCalibrationY * magResolution - magnetometerBias[1];
    magMeasurements.y = (float) imuData[6] * fMagCalibrationX * magResolution - magnetometerBias[0];
    magMeasurements.z = ((float) imuData[8] * fMagCalibrationZ * magResolution - magnetometerBias[2]);

    magMeasurements.y *= magnetometerSensitivity[1];
    magMeasurements.x *= magnetometerSensitivity[0];
    magMeasurements.z *= magnetometerSensitivity[2];

    orientationFilterUpdateImu(&orientationFilterCtx, &accMeasurements, &gyroMeasurements, false);
    //orientationFilterUpdate(&orientationFilterCtx, &accMeasurements, &gyroMeasurements, &magMeasurements, false);
    getSetPoint(&setPoint);
    orientationEstimate.orientation.w = orientationFilterCtx.filterEstimate.w;
    orientationEstimate.orientation.x = orientationFilterCtx.filterEstimate.x;
    orientationEstimate.orientation.y = orientationFilterCtx.filterEstimate.y;
    orientationEstimate.orientation.z = orientationFilterCtx.filterEstimate.z;
    flightControllerUpdate(&setPoint, &orientationEstimate, &motorOutput);
    timersHalSetPwmFastChannel0(motorOutput.motorFrontLeft);
    timersHalSetPwmFastChannel1(motorOutput.motorBackLeft);
    timersHalSetPwmFastChannel2(motorOutput.motorBackRight);
    timersHalSetPwmFastChannel3(motorOutput.motorFrontRight);
    isImuFrameReady = true;
}

static void halInit(void)
{
    halCommonInit();
    gpioHalInit();
    timersHalInit(onTimerEvent);
    delayMs(100);
    buttonHalInit();
    adcHalInit();
    spiHalImuInit(onImuFrameReady);
    sensorSystemInit();
}

static void btInit(bool isSetupMode)
{
    #define BT_DEFAULT_UART_SPEED   38400
    #define BT_MAX_UART_SPEED       1382400
    #define BT_GPIO_REACTION_DELAY  30
    #define BT_MAX_UART_SPEED_AT    "AT+UART=1382400,1,0\r\n"
    #define BT_DEVICE_NAME_AT       "AT+NAME=Quad\r\n"

    if (isSetupMode) {
        gpioHalEnableBt(false);
        gpioHalEnableBtAt(true);
        delayMs(BT_GPIO_REACTION_DELAY);
        gpioHalEnableBt(true);
        uartBtInit(BT_DEFAULT_UART_SPEED);
        uartBtDmaStartStopRx(true);

        // Set configuration for name and max speed
        isBtTxReady = false;
        uartBtDmaTx((uint8_t *) BT_MAX_UART_SPEED_AT, sizeof(BT_MAX_UART_SPEED_AT) - 1, onBtTxReady);
        while (!isBtTxReady);

        isBtTxReady = false;
        uartBtDmaTx((uint8_t *) BT_DEVICE_NAME_AT, sizeof(BT_DEVICE_NAME_AT) - 1, onBtTxReady);
        while (!isBtTxReady);

        gpioHalEnableBt(false);
        gpioHalEnableBtAt(false);
        delayMs(BT_GPIO_REACTION_DELAY);
        gpioHalEnableBt(true);
    } else {
        gpioHalEnableBt(true);
        uartBtInit(BT_MAX_UART_SPEED);
        uartBtDmaStartStopRx(true);
        isBtTxReady = true;
    }
}


static void flightControllerStart()
{
    #define OF_FILTER_COEF_ALPHA  0.0002f
    #define OF_FILTER_COEF_BETA   0.0002f

    #define OF_FILTER_COEF_GAMMA  0.00000002f
    #define OF_FILTER_COEF_THETA  0.0002f
    orientationFilterInitialize(&orientationFilterCtx, 1.0f / (float) SENSOR_SYSTEM_UPDATE_RATE, OF_FILTER_COEF_ALPHA, OF_FILTER_COEF_ALPHA, OF_FILTER_COEF_GAMMA, OF_FILTER_COEF_THETA);
    flightControllerInit(SENSOR_SYSTEM_UPDATE_RATE);

    spiHalImuReadOutStart();

}

static void btPrintOutput(void)
{
    #define QUAD_CLR  "\x01D"
    isBtTxReady = false;
    uartBtDmaTx((uint8_t *) QUAD_CLR, sizeof(QUAD_CLR) - 1, onBtTxReady);
    while (!isBtTxReady);

    #define QUAD_READY_MESSAGE  "Quad ready!\x01E"
    isBtTxReady = false;
    uartBtDmaTx((uint8_t *) QUAD_READY_MESSAGE, sizeof(QUAD_READY_MESSAGE) - 1, onBtTxReady);
    while (!isBtTxReady);

    #define QUAD_MESSAGE_0  "Left -> throttle\x01E"
    isBtTxReady = false;
    uartBtDmaTx((uint8_t *) QUAD_MESSAGE_0, sizeof(QUAD_MESSAGE_0) - 1, onBtTxReady);
    while (!isBtTxReady);

    #define QUAD_MESSAGE_1  "Right -> direction\x01E"
    isBtTxReady = false;
    uartBtDmaTx((uint8_t *) QUAD_MESSAGE_1, sizeof(QUAD_MESSAGE_1) - 1, onBtTxReady);
    while (!isBtTxReady);

}

int main(void)
{
    SystemCoreClockUpdate();
    halInit();
    btInit(false);

    ledControllerEnableLeds();
    ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_GREEN_LEFT, 100);
    ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_GREEN_RIGHT, 100);
    ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_RED_LEFT, 250);
    ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_RED_RIGHT, 250);

    delayMs(1000);
    gpioHalEnablePower(true);
    while (buttonHalGetState());
    delayMs(1000);
    uint32_t ledBrightness = 0, batteryPeriodCounter = 0, powerOnCntr = 0;
    bool dn = false;
    sensorSystemCalibrate(magnetometerBias, magnetometerSensitivity, false);
    uint8_t batteryState[] = {'\x01F', 0x09, 0x09};
    btPrintOutput();
    btPrintOutput();
    flightControllerStart();
    for (;;) {
        protocolProcessBuffer();
        if (isTimerEvent) {
            if (!isPowerEnabled) {
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_GREEN_LEFT, ledBrightness >> 1);
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_GREEN_RIGHT, ledBrightness >> 1);
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_RED_LEFT, ledBrightness);
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_RED_RIGHT, ledBrightness);
                if (dn) {
                    ledBrightness -= 50;
                    if (ledBrightness <= 50) {
                        dn = false;
                    }
                } else {
                    ledBrightness += 50;
                    if (ledBrightness >= 1000) {
                        dn = true;
                    }
                }
            }
            if (batteryPeriodCounter++ == 25) {
                batteryPeriodCounter = 0;
                while (!isBtTxReady);
                isBtTxReady = false;
                uint32_t batteryValue = (adcHalGetValue() * ADC_REF_VAL_MV / ADC_MAX_VAL) << 1;
                if (batteryValue < ADC_REF_VAL_MV)
                    batteryValue = 0;
                else
                    batteryValue -= ADC_REF_VAL_MV;
                batteryValue = batteryValue * 99 / (LI_ION_FULL_VOLTAGE - ADC_REF_VAL_MV);
                batteryState[1] =  ((batteryValue / 10) & 0x0F);
                batteryState[2] = ((batteryValue % 10) & 0x0F);
                uartBtDmaTx((uint8_t *) batteryState, sizeof(batteryState), onBtTxReady);
            }

            if ((protocolGetJoystickValues()->joyDnLeftYaxisB1 & 0x1000)  && (protocolGetJoystickValues()->joyDnRightXaxisB2 & 0x1000)) {
                if (powerOnCntr != 11)
                    powerOnCntr++;
            } else {
                powerOnCntr = 0;
            }
            if (powerOnCntr == 10) {
                powerOnCntr = 11;
                isPowerEnabled = !isPowerEnabled;
                timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_0, isPowerEnabled);
                timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_1, isPowerEnabled);
                timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_2, isPowerEnabled);
                timersHalEnablePwmChannel(TIMERS_HAL_PWM_CHANNEL_3, isPowerEnabled);

                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_GREEN_LEFT, 200);
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_GREEN_RIGHT, 200);
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_RED_LEFT, 500);
                ledControllerSetLedLevel(LED_CONTROLLER_CHANNEL_RED_RIGHT, 500);
               // clearFlash();
            }


            isTimerEvent = false;
        }
        if (buttonHalGetState()) {
            gpioHalEnablePower(false);

        }
    }
}

/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include "stm32f0xx_conf.h"
#include "HalCommon.h"
#include "HalGpio.h"
#include "HalUart.h"
#include "HalI2c.h"
#include "HalAdc.h"
#include "Timer.h"

#define SET_Rf_PARAMS   0

typedef enum {
    StartByte,
    CmdByte,
} ProtocolState;


static uint8_t batteryLevel = 0;
static uint32_t cmdTimer = 0;
static bool connectionStatus = false;

void appProcessProtocolByte(uint8_t cmdByte)
{
    cmdTimer = 0;
    batteryLevel = cmdByte;
}

void appProcessCmdBuffer(uint32_t lastByte)
{
    static uint32_t firstByte = 0;
    uint32_t byteCounter = firstByte;

    while (byteCounter != lastByte) {
        appProcessProtocolByte(uartCmdInBuffer[byteCounter]);
        byteCounter = (byteCounter + 1) & UART_CMD_BUFF_MASK;
    }

    firstByte = lastByte;
}

volatile bool isRfTxReady = false;

void rfTxReady()
{
    isRfTxReady = true;
}

static volatile bool isLcdRefreshNeeded = true;

void timerCb(void)
{
    if (cmdTimer++ >= 1000) {
        cmdTimer = 1000;
    }

    isLcdRefreshNeeded = true;
}

static volatile uint32_t ovrCntr = 0;

uint32_t adcValues[ADC_CHANNEL_COUNT];
volatile bool adcValuesReady = false;
void adcDataReady(const uint16_t data[], uint32_t length)
{
    if (adcValuesReady == false) {

        uint32_t bcnt = 0, ccnt = 0;

        for (ccnt = 0; ccnt < ADC_CHANNEL_COUNT; ccnt++) {
            adcValues[ccnt] = 0;
        }

        for (bcnt = 0; bcnt < ADC_BUFF_SIZE; bcnt++) {
            for (ccnt = 0; ccnt < ADC_CHANNEL_COUNT; ccnt++) {
                adcValues[ccnt] += data[bcnt * ADC_CHANNEL_COUNT + ccnt];
            }
        }

        for (ccnt = 0; ccnt < ADC_CHANNEL_COUNT; ccnt++) {
            adcValues[ccnt] >>= 6;
        }

        adcValuesReady = true;
    } else {
        ovrCntr++;
    }
}

uint32_t getButtonState(uint32_t adcValue)
{
    #define JOU_LEFT    842
    #define JOU_RIGHT   958
    #define UP          1107
    #define DOWN        1298
    #define LEFT        1983
    #define RIGHT       1567
    #define MARGIN      15

    if (adcValue < (JOU_LEFT + MARGIN) && adcValue > (JOU_LEFT - MARGIN)) {
        return 0x01;
    }
    if (adcValue < (JOU_RIGHT + MARGIN) && adcValue > (JOU_RIGHT - MARGIN)) {
        return 0x02;
    }
    if (adcValue < (UP + MARGIN) && adcValue > (UP - MARGIN)) {
        return 0x03;
    }
    if (adcValue < (DOWN + MARGIN) && adcValue > (DOWN - MARGIN)) {
        return 0x04;
    }
    if (adcValue < (LEFT + MARGIN) && adcValue > (LEFT - MARGIN)) {
        return 0x5;
    }
    if (adcValue < (RIGHT + MARGIN) && adcValue > (RIGHT - MARGIN)) {
        return 0x6;
    }

    return 0;
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

void packRcData(uint8_t buffer[], uint16_t joyData[], uint8_t buttonState)
{
    buffer[0] = ((joyData[0] >> 4) | 0x80);
    buffer[1] = (((joyData[0] << 4) | (joyData[1] >> 8)) & ~0x80);
    buffer[2] = (joyData[1] & ~0x80);
    buffer[3] = ((joyData[2] >> 4) & ~0x80);
    buffer[4] = (((joyData[2] << 4) | (joyData[3] >> 8)) & ~0x80);
    buffer[5] = (joyData[3] & ~0x80);
    buffer[6] = ((joyData[0] >> 11) & 0x01) | ((joyData[0] >> 2) & 0x02) | ((joyData[1] >> 5) & 0x04) | ((joyData[2] >> 8) & 0x08) | ((joyData[2] << 1) & 0x10) | ((joyData[3] >> 2) & 0x20);
    buffer[7] = buttonState;
    buffer[8] = crc7(buffer, 8);
}

uint32_t batteryVoltageRc, batteryVoltageQ, buttonState;
uint16_t joyData[4];
uint8_t rcData[9];

int main(void)
{
    halCommonInit();
    halGpioInit();

    #if (SET_Rf_PARAMS == 1)
    halGpioEnableRfAt(true);

    volatile uint32_t cntr = 0;
    for (cntr = 0; cntr < 30000; cntr ++) {

    }

    uartRfInit(9600);

    uartRfDmaStartStopRx(true);

    // Set configuration
    isRfTxReady = false;
    uartRfDmaTx((uint8_t *) "AT+B115200", sizeof("AT+B115200") - 1, rfTxReady);
    while (!isRfTxReady);

    halGpioEnableRfAt(false);

    for (cntr = 0; cntr < 30000; cntr ++) {

    }
    #else
    uartRfInit(115200);
    uartRfDmaStartStopRx(true);
    isRfTxReady = true;
    #endif

    volatile uint32_t cntr = 0;
    for (cntr = 0; cntr < 300000; cntr ++) {

    }

    uartRfDmaStartStopRx(true);
    halI2cLcdInit();
    timerInit(timerCb);
    #define BAT_START  128
    char batRc[] = {BAT_START, 0};
    char batQ[] = {BAT_START, 0};

    halI2cLcdPrintString(15, 0, "Rc   battery: ");
    halI2cLcdPrintString(108, 0, batRc);
    halI2cLcdPrintString(15, 2, "Quad battery: ");
    halI2cLcdPrintString(108, 2, batQ);
    halI2cLcdPrintString(15, 4, "Ch: 0");
    halI2cLcdPrintString(80, 4, "Conn: ");
    halI2cSetConn(false);

    halI2cInitJoyBars();
    halI2cInitButton();
    halI2cLcdRefresh();

    halAdcInit(adcDataReady);

    for (;;) {
        if (adcValuesReady) {

            batteryVoltageRc = (adcValues[0] * 6750) >> 12;
            batteryVoltageRc = (batteryVoltageRc - 3300) * 6 / 850;

            if (batteryVoltageRc > 6) {
                batteryVoltageRc = 6;
            }

            batteryVoltageQ = (batteryLevel * 20 - 3300) * 6 / 850;
            if (batteryVoltageQ > 6) {
                batteryVoltageQ = 6;
            }
            batRc[0] = BAT_START + batteryVoltageRc;
            halI2cLcdPrintString(108, 0, batRc);
            batQ[0] = BAT_START + batteryVoltageQ;
            halI2cLcdPrintString(108, 2, batQ);

            if (cmdTimer >= 1000) {
                connectionStatus = false;
            } else {
                connectionStatus = true;
            }

            halI2cSetConn(connectionStatus);
            buttonState = adcValues[5];
            joyData[0] = adcValues[2];
            joyData[1] = adcValues[3];
            joyData[2] = adcValues[4];
            joyData[3] = adcValues[1];

            buttonState = getButtonState(buttonState);
            halI2cSetJoyBars(joyData[0], joyData[1], joyData[2], joyData[3]);
            halI2cSetButton(buttonState);
            packRcData(rcData, joyData, buttonState);

            if (isRfTxReady) {
                uartRfDmaTx(rcData, sizeof(rcData), rfTxReady);
                isRfTxReady = false;
            }
            adcValuesReady = false;
        }

        appProcessCmdBuffer(uartRfGetCurrentRxBuffIdx());

        if (isLcdRefreshNeeded) {
            halI2cLcdRefreshMin();
            isLcdRefreshNeeded = false;
        }

    }
}

