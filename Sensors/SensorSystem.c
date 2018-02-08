#include "SensorSystem.h"
#include "OrientationFilter.h"
#include "HalI2c.h"

#include <math.h>
#include <stddef.h>

#define REG_ADDR_SMPLRT_DIV         0x19
#define REG_ADDR_CONFIG             0x1A
#define REG_ADDR_GYRO_CONFIG        0x1B
#define REG_ADDR_ACCEL_CONFIG       0x1C
#define REG_ADDR_INT_PIN_CFG        0x37
#define REG_ADDR_INT_ENABLE         0x38
#define REG_ADDR_INT_STATUS         0x3A

#define REG_ADDR_ACCEL_XOUT_H       0x3B
#define REG_ADDR_ACCEL_XOUT_L       0x3C
#define REG_ADDR_ACCEL_YOUT_H       0x3D
#define REG_ADDR_ACCEL_YOUT_L       0x3E
#define REG_ADDR_ACCEL_ZOUT_H       0x3F
#define REG_ADDR_ACCEL_ZOUT_L       0x40
#define REG_ADDR_TEMP_OUT_H         0x41
#define REG_ADDR_TEMP_OUT_L         0x42
#define REG_ADDR_GYRO_XOUT_H        0x43
#define REG_ADDR_GYRO_XOUT_L        0x44
#define REG_ADDR_GYRO_YOUT_H        0x45
#define REG_ADDR_GYRO_YOUT_L        0x46
#define REG_ADDR_GYRO_ZOUT_H        0x47
#define REG_ADDR_GYRO_ZOUT_L        0x48

#define REG_ADDR_SIGNAL_PATH_RES    0x68
#define REG_ADDR_USER_CTRL          0x6A
#define REG_ADDR_PWR_MGMT_1         0x6B
#define REG_ADDR_PWR_MGMT_2         0x6C

#define REG_ADDR_XG_OFFS_TC         0x00
#define REG_ADDR_YG_OFFS_TC         0x01
#define REG_ADDR_ZG_OFFS_TC         0x02
#define REG_ADDR_X_FINE_GAIN        0x03
#define REG_ADDR_Y_FINE_GAIN        0x04
#define REG_ADDR_Z_FINE_GAIN        0x05
#define REG_ADDR_XA_OFFS_H          0x06
#define REG_ADDR_XA_OFFS_L_TC       0x07
#define REG_ADDR_YA_OFFS_H          0x08
#define REG_ADDR_YA_OFFS_L_TC       0x09
#define REG_ADDR_ZA_OFFS_H          0x0A
#define REG_ADDR_ZA_OFFS_L_TC       0x0B
#define REG_ADDR_SELF_TEST_X        0x0D
#define REG_ADDR_SELF_TEST_Y        0x0E
#define REG_ADDR_SELF_TEST_Z        0x0F
#define REG_ADDR_SELF_TEST_A        0x10
#define REG_ADDR_XG_OFFS_USRH       0x13
#define REG_ADDR_XG_OFFS_USRL       0x14
#define REG_ADDR_YG_OFFS_USRH       0x15
#define REG_ADDR_YG_OFFS_USRL       0x16
#define REG_ADDR_ZG_OFFS_USRH       0x17
#define REG_ADDR_ZG_OFFS_USRL       0x18

#define I2C_PERIPH_IMU_ADDRESS      (0x68 << 1)

#define DEG_180                     180.0f
#define M_PI_F                      3.1415926535897932384626433832795f

#define IMU_SAMPLING_FREQUENCY_MAX  1000
#define IMU_MAX_OUTPUT_VALUE        32767.0f
#define IMU_DESIRED_G_ACC_VALUE     4096

const enum ImuAccRange {
    IMU_ACC_SENSITIVITY_2G,
    IMU_ACC_SENSITIVITY_4G,
    IMU_ACC_SENSITIVITY_8G,
    IMU_ACC_SENSITIVITY_16G,
    IMU_ACC_SENSITIVITY_COUNT,
} imuAccRange = IMU_ACC_SENSITIVITY_8G;

const enum ImuGyroRange {
    IMU_GYRO_SENSITIVITY_250DEG,
    IMU_GYRO_SENSITIVITY_500DEG,
    IMU_GYRO_SENSITIVITY_1000DEG,
    IMU_GYRO_SENSITIVITY_2000DEG,
    IMU_GURO_SENSITIVITY_COUNT,
} imuGyroRange = IMU_GYRO_SENSITIVITY_2000DEG;

const enum ImuDlpf {
    IMU_DLPF_260_250_HZ,
    IMU_DLPF_184_188_HZ,
    IMU_DLPF_94_98_HZ,
    IMU_DLPF_44_42_HZ,
    IMU_DLPF_21_20_HZ,
    IMU_DLPF_10_10_HZ,
    IMU_DLPF_5_5_HZ,
} imuDlpf = IMU_DLPF_44_42_HZ;

static SensorSystemUpdateCallback sensorSystemUpdateCallback;
static OrientationFilterCtx orientationFilterCtx;
static FlightControllerOrientationEstimate orientationEstimate;
static bool isUpdateEventEnabled = false;

static const float imuAccRangeValues[IMU_ACC_SENSITIVITY_COUNT] = {2.0f, 4.0f, 8.0f, 16.0f};
static const float imuGyroRangeValues[IMU_GURO_SENSITIVITY_COUNT] = {250.0f, 500.0f, 1000.0, 2000.0f};

static void setXAccelOffset(int16_t offset)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_XA_OFFS_H, offset >> 8);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_XA_OFFS_H + 1, offset);
}

static void setYAccelOffset(int16_t offset)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_YA_OFFS_H, offset >> 8);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_YA_OFFS_H + 1, offset);
}

static void setZAccelOffset(int16_t offset)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ZA_OFFS_H, offset >> 8);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ZA_OFFS_H + 1, offset);
}

static void setXGyroOffset(int16_t offset)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_XG_OFFS_USRH, offset >> 8);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_XG_OFFS_USRH + 1, offset);
}

static void setYGyroOffset(int16_t offset)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_YG_OFFS_USRH, offset >> 8);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_YG_OFFS_USRH + 1, offset);
}

static void setZGyroOffset(int16_t offset)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ZG_OFFS_USRH, offset >> 8);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ZG_OFFS_USRH + 1, offset);
}

static void unpackImuFrame(const uint8_t frame[], uint16_t unpackedValues[])
{
    uint8_t *temp = (uint8_t *) unpackedValues;

    temp[0] = frame[1]; temp[1] = frame[0];
    temp[2] = frame[3]; temp[3] = frame[2];
    temp[4] = frame[5]; temp[5] = frame[4];

    temp[6] = frame[9]; temp[7] = frame[8];
    temp[8] = frame[11]; temp[9] = frame[10];
    temp[10] = frame[13]; temp[11] = frame[12];
}

static void eulerFromQuaternion(void)
{
    float gx = 2.0f * (orientationEstimate.orientation.x * orientationEstimate.orientation.z - orientationEstimate.orientation.w * orientationEstimate.orientation.y);
    float gy = 2.0f * (orientationEstimate.orientation.w * orientationEstimate.orientation.x + orientationEstimate.orientation.y * orientationEstimate.orientation.z);
    float gz = orientationEstimate.orientation.w * orientationEstimate.orientation.w - orientationEstimate.orientation.x * orientationEstimate.orientation.x -
                orientationEstimate.orientation.y * orientationEstimate.orientation.y + orientationEstimate.orientation.z * orientationEstimate.orientation.z;

    if (gx > 1.0f) {
        gx = 1.0f;
    }
    if (gx < -1.0f) {
        gx = -1.0f;
    }

    orientationEstimate.currentYaw = atan2f(2.0f * (orientationEstimate.orientation.w * orientationEstimate.orientation.z +
                                                    orientationEstimate.orientation.x * orientationEstimate.orientation.y),
                                                    orientationEstimate.orientation.w * orientationEstimate.orientation.w +
                                                    orientationEstimate.orientation.x * orientationEstimate.orientation.x -
                                                    orientationEstimate.orientation.y * orientationEstimate.orientation.y -
                                                    orientationEstimate.orientation.z * orientationEstimate.orientation.z) * DEG_180 / M_PI_F;

    orientationEstimate.currentPitch = asinf(gx) * DEG_180 / M_PI_F;
    orientationEstimate.currentRoll = atan2f(gy, gz) * DEG_180 / M_PI_F;
}

static void onIntEvent(void)
{
    halI2cReadWithDma(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ACCEL_XOUT_H, 14);
}
    static OfVector3 accMeasurements, gyroMeasurements;
static void onI2cDmaReadyEvent(void)
{

    static int16_t imuData[6];
    uint32_t imuDatalength;

    uint8_t *data = halI2cGetReadBuffer(&imuDatalength);

    unpackImuFrame(data, (uint16_t *) imuData);

    accMeasurements.x = (float) imuData[0] * imuAccRangeValues[imuAccRange] / IMU_MAX_OUTPUT_VALUE;
    accMeasurements.y = (float) imuData[1] * imuAccRangeValues[imuAccRange] / IMU_MAX_OUTPUT_VALUE;
    accMeasurements.z = (float) imuData[2] * imuAccRangeValues[imuAccRange] / IMU_MAX_OUTPUT_VALUE;
    gyroMeasurements.x = (float) imuData[3] * imuGyroRangeValues[imuGyroRange] * M_PI_F / IMU_MAX_OUTPUT_VALUE / DEG_180;
    gyroMeasurements.y = (float) imuData[4] * imuGyroRangeValues[imuGyroRange] * M_PI_F / IMU_MAX_OUTPUT_VALUE / DEG_180;
    gyroMeasurements.z = (float) imuData[5] * imuGyroRangeValues[imuGyroRange] * M_PI_F / IMU_MAX_OUTPUT_VALUE / DEG_180;

    orientationFilterUpdateImu(&orientationFilterCtx, &accMeasurements, &gyroMeasurements, false);

    orientationEstimate.orientation.w = orientationFilterCtx.filterEstimate.w;
    orientationEstimate.orientation.x = orientationFilterCtx.filterEstimate.x;
    orientationEstimate.orientation.y = orientationFilterCtx.filterEstimate.y;
    orientationEstimate.orientation.z = orientationFilterCtx.filterEstimate.z;

    eulerFromQuaternion();

    if (sensorSystemUpdateCallback != NULL && isUpdateEventEnabled) {
        sensorSystemUpdateCallback();
    }
}

static void setupImu(uint8_t sampleRateDiv, enum ImuDlpf dlpf, enum ImuGyroRange gyroFsel, enum ImuAccRange accFsel)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_PWR_MGMT_1, 0x01);

    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_SMPLRT_DIV, sampleRateDiv);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_CONFIG, dlpf);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_GYRO_CONFIG, gyroFsel << 3);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ACCEL_CONFIG, accFsel << 3);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_INT_ENABLE, 0x00);
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_INT_PIN_CFG, 0x10);
}

static void setupImuInt(bool isEnabled)
{
    halI2cWrite(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_INT_ENABLE, isEnabled ? 0x01 : 0x00);
    halI2cRead(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_INT_STATUS);
}

static uint32_t absI(int32_t value)
{
    return value < 0 ? -value : value;
}

static void getImuValues(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t imuData[14];
    int16_t imuValues[6];

    halI2cReadBuf(I2C_PERIPH_IMU_ADDRESS, REG_ADDR_ACCEL_XOUT_H, imuData, sizeof(imuData));
    unpackImuFrame(imuData, (uint16_t *) imuValues);

    *ax = imuValues[0];
    *ay = imuValues[1];
    *az = imuValues[2];

    *gx = imuValues[3];
    *gy = imuValues[4];
    *gz = imuValues[5];
}

static void getMeanValues(int32_t *meanAx, int32_t *meanAy, int32_t *meanAz, int32_t *meanGx, int32_t *meanGy, int32_t *meanGz)
{
    #define AVERAGING_FACTOR    8000
    int16_t ax, ay, az, gx, gy, gz;
    uint32_t i = 0;

    *meanAx = 0;
    *meanAy = 0;
    *meanAz = 0;
    *meanGx = 0;
    *meanGy = 0;
    *meanGz = 0;

    while (i++ < AVERAGING_FACTOR) {
        getImuValues(&ax, &ay, &az, &gx, &gy, &gz);
        *meanAx += ax;
        *meanAy += ay;
        *meanAz += az;
        *meanGx += gx;
        *meanGy += gy;
        *meanGz += gz;
    }

    *meanAx /= AVERAGING_FACTOR;
    *meanAy /= AVERAGING_FACTOR;
    *meanAz /= AVERAGING_FACTOR;
    *meanGx /= AVERAGING_FACTOR;
    *meanGy /= AVERAGING_FACTOR;
    *meanGz /= AVERAGING_FACTOR;
}

static void calibrationProcess(int32_t *gxOffset, int32_t *gyOffset, int32_t *gzOffset, int32_t *axOffset, int32_t *ayOffset, int32_t *azOffset)
{
    int32_t meanAx, meanAy, meanAz, meanGx, meanGy, meanGz;
    bool isXaReady = false, isYaReady = false, isZaReady = false, isXgReady = false, isYgReady = false, isZgReady = false;

    getMeanValues(&meanAx, &meanAy, &meanAz, &meanGx, &meanGy, &meanGz);
    *gxOffset = -(meanGx >> 2);
    *gyOffset = -(meanGy >> 2);
    *gzOffset = -(meanGz >> 2);

    *axOffset = -(meanAx >> 3);
    *ayOffset = -(meanAy >> 3);
    *azOffset = (IMU_DESIRED_G_ACC_VALUE - meanAz) >> 3;

    for (;;) {
        setXGyroOffset(*gxOffset);
        setYGyroOffset(*gyOffset);
        setZGyroOffset(*gzOffset);

        setXAccelOffset(*axOffset);
        setYAccelOffset(*ayOffset);
        setZAccelOffset(*azOffset);

        getMeanValues(&meanAx, &meanAy, &meanAz, &meanGx, &meanGy, &meanGz);

        if (absI(meanGx) <= 1) {
            isXaReady = true;
        } else {
            *gxOffset -= (meanGx >> 1);
        }

        if (absI(meanGy) <= 1) {
            isYaReady = true;
        } else {
            *gyOffset -= (meanGy >> 1);
        }

        if (absI(meanGz) <= 1) {
            isZaReady = true;
        } else {
            *gzOffset -= (meanGz >> 1);
        }

        if (absI(meanAx) <= 1) {
            isXgReady = true;
        } else {
            *axOffset -= meanAx >> 1;
        }

        if (absI(meanAy) <= 1) {
            isYgReady = true;
        } else {
            *ayOffset -= meanAy >> 1;
        }

        if (absI(IMU_DESIRED_G_ACC_VALUE - meanAz) <= 1) {
            isZgReady = true;
        } else {
            *azOffset += (IMU_DESIRED_G_ACC_VALUE - meanAz) >> 1;
        }

        if (isXaReady && isYaReady && isZaReady && isXgReady && isYgReady && isZgReady) {
            break;
        }
    }
}

void sensorSystemInit(SensorSystemUpdateCallback sensorSystemUpdateCb, uint32_t updateRate)
{
    sensorSystemUpdateCallback = sensorSystemUpdateCb;
    halI2cInit(onI2cDmaReadyEvent, onIntEvent);
    setupImu(IMU_SAMPLING_FREQUENCY_MAX / SENSOR_SYSTEM_UPDATE_RATE - 1, imuDlpf, imuGyroRange, imuAccRange);

    #define OF_FILTER_COEF_ALPHA  0.0001f
    #define OF_FILTER_COEF_BETA   0.0001f
    #define OF_FILTER_COEF_GAMMA  0.00000001f
    #define OF_FILTER_COEF_THETA  0.0001f

    orientationFilterInitialize(&orientationFilterCtx, 1.0f / (float) updateRate, OF_FILTER_COEF_ALPHA, OF_FILTER_COEF_ALPHA, OF_FILTER_COEF_GAMMA, OF_FILTER_COEF_THETA);

    #undef OF_FILTER_COEF_ALPHA
    #undef OF_FILTER_COEF_BETA
    #undef OF_FILTER_COEF_GAMMA
    #undef OF_FILTER_COEF_THETA
}

void sensorSystemCalibrate(bool isBiasUpdateNeeded)
{
    int32_t gxOffset = 17, gyOffset = 78, gzOffset = 29;
    int32_t axBias = -4115, ayBias = -241, azBias = 862;

    if (isBiasUpdateNeeded) {
        calibrationProcess(&gxOffset, &gyOffset, &gzOffset, &axBias, &ayBias, &azBias);
    } else {
        setXGyroOffset(gxOffset);
        setYGyroOffset(gyOffset);
        setZGyroOffset(gzOffset);
        setXAccelOffset(axBias);
        setYAccelOffset(ayBias);
        setZAccelOffset(azBias);
    }
}

void sensorSystemGetCurrentOrientation(FlightControllerOrientationEstimate *flightControllerOrientationEstimate)
{
    if (flightControllerOrientationEstimate != NULL) {
        *flightControllerOrientationEstimate = orientationEstimate;
    }
}

void sensorSystemStartUpdateEvent(bool isUpdateEnabled)
{
    setupImuInt(isUpdateEnabled);
    isUpdateEventEnabled = isUpdateEnabled;
}
