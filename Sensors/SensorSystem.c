#include "SensorSystem.h"
#if 0
#include "SpiHal.h"
#include "stm32f4xx_conf.h"

static void imuWriteBuff(uint8_t address, const uint8_t data[], uint32_t size)
{
    uint32_t cntr = 0;
    for (cntr = 0; cntr < size; cntr++)
        spiHalImuWriteReg(address++, data[cntr]);
}

static void imuReadBuff(uint8_t address, uint8_t data[], uint32_t size)
{
    uint32_t cntr = 0;
    for (cntr = 0; cntr < size; cntr++)
        data[cntr] = spiHalImuReadReg(address++);
}

uint8_t magCalibrationX;
uint8_t magCalibrationY;
uint8_t magCalibrationZ;

float fMagCalibrationX;
float fMagCalibrationY;
float fMagCalibrationZ;
void sensorSystemInit(void)
{
    const uint8_t imuSensorConfig[] = {0x01, 0x01, 0x18, 0x10, 0x01};
    const uint8_t imuIntConfig[] = {0x10, 0x01};
    const uint8_t imuPwrConfig[] = {0x30, 0x00, 0x00};

    const uint8_t imuSlaveI2cConfig1[] = {0x09};
    const uint8_t imuSlaveI2cConfig2[] = {0x0C, 0x0A, 0x16, 0x80};
    const uint8_t imuSlaveI2cConfig4[] = {0x0C, 0x0A, 0x0F, 0x80};
    const uint8_t imuSlaveI2cConfig3[] = {0x8C, 0x03, 0x87};

    const uint8_t imuSlaveI2cConfig5[] = {0x8C, 0x10, 0x00, 0x80};
    const uint8_t imuSlaveI2cConfig6[] = {0x8C, 0x11, 0x00, 0x80};
    const uint8_t imuSlaveI2cConfig7[] = {0x8C, 0x12, 0x00, 0x80};

    imuWriteBuff(IMU_REG_USER_CTRL, imuPwrConfig, sizeof(imuPwrConfig));
    imuWriteBuff(IMU_REG_SMPLRT_DIV, imuSensorConfig, sizeof(imuSensorConfig));
    imuWriteBuff(IMU_REG_INT_PIN_CFG, imuIntConfig, sizeof(imuIntConfig));

    imuWriteBuff(IMU_REG_I2C_MST_CTRL, imuSlaveI2cConfig1, sizeof(imuSlaveI2cConfig1));
    uint8_t i2cStatus = 0;

    imuWriteBuff(IMU_REG_I2C_SLV4_ADDR, imuSlaveI2cConfig4, sizeof(imuSlaveI2cConfig4));
    do {
        imuReadBuff(IMU_REG_I2C_MST_STATUS, &i2cStatus, sizeof(i2cStatus));
    } while (!(i2cStatus & (1 << 6)));

    imuWriteBuff(IMU_REG_I2C_SLV4_ADDR, imuSlaveI2cConfig5, sizeof(imuSlaveI2cConfig5));
    do {
        imuReadBuff(IMU_REG_I2C_MST_STATUS, &i2cStatus, sizeof(i2cStatus));
    } while (!(i2cStatus & (1 << 6)));
    imuReadBuff(IMU_REG_I2C_SLV4_DI, &magCalibrationX, sizeof(magCalibrationX));

    imuWriteBuff(IMU_REG_I2C_SLV4_ADDR, imuSlaveI2cConfig6, sizeof(imuSlaveI2cConfig6));
    do {
        imuReadBuff(IMU_REG_I2C_MST_STATUS, &i2cStatus, sizeof(i2cStatus));
    } while (!(i2cStatus & (1 << 6)));
    imuReadBuff(IMU_REG_I2C_SLV4_DI, &magCalibrationY, sizeof(magCalibrationY));

    imuWriteBuff(IMU_REG_I2C_SLV4_ADDR, imuSlaveI2cConfig7, sizeof(imuSlaveI2cConfig7));
    do {
        imuReadBuff(IMU_REG_I2C_MST_STATUS, &i2cStatus, sizeof(i2cStatus));
    } while (!(i2cStatus & (1 << 6)));
    imuReadBuff(IMU_REG_I2C_SLV4_DI, &magCalibrationZ, sizeof(magCalibrationZ));

    imuWriteBuff(IMU_REG_I2C_SLV4_ADDR, imuSlaveI2cConfig2, sizeof(imuSlaveI2cConfig2));
    do {
        imuReadBuff(IMU_REG_I2C_MST_STATUS, &i2cStatus, sizeof(i2cStatus));
    } while (!(i2cStatus & (1 << 6)));

    imuWriteBuff(IMU_REG_I2C_SLV0_ADDR, imuSlaveI2cConfig3, sizeof(imuSlaveI2cConfig3));

    fMagCalibrationX = (((float) magCalibrationX - 128.0f) / 256.0f + 1.0f);
    fMagCalibrationY = (((float) magCalibrationY - 128.0f) / 256.0f + 1.0f);
    fMagCalibrationZ = (((float) magCalibrationZ - 128.0f) / 256.0f + 1.0f);
}
/*
static int16_t getXAccelOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = spiHalImuReadReg(IMU_REG_XA_OFFSET_H);
    buffer[1] = spiHalImuReadReg(IMU_REG_XA_OFFSET_L);

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

static void setXAccelOffset(int16_t offset)
{
    spiHalImuWriteReg(IMU_REG_XA_OFFSET_H, offset >> 8);
    spiHalImuWriteReg(IMU_REG_XA_OFFSET_L, offset);
}

static int16_t getYAccelOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = spiHalImuReadReg(IMU_REG_YA_OFFSET_H);
    buffer[1] = spiHalImuReadReg(IMU_REG_YA_OFFSET_L);

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

static void setYAccelOffset(int16_t offset)
{
    spiHalImuWriteReg(IMU_REG_YA_OFFSET_H, offset >> 8);
    spiHalImuWriteReg(IMU_REG_YA_OFFSET_L, offset);
}

static int16_t getZAccelOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = spiHalImuReadReg(IMU_REG_ZA_OFFSET_H);
    buffer[1] = spiHalImuReadReg(IMU_REG_ZA_OFFSET_L);

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

static void setZAccelOffset(int16_t offset)
{
    spiHalImuWriteReg(IMU_REG_ZA_OFFSET_H, offset >> 8);
    spiHalImuWriteReg(IMU_REG_ZA_OFFSET_L, offset);
}

static int16_t getXGyroOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = spiHalImuReadReg(IMU_REG_XG_OFFSET_H);
    buffer[1] = spiHalImuReadReg(IMU_REG_XG_OFFSET_L);

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
*/
static void setXGyroOffset(int16_t offset)
{
    spiHalImuWriteReg(IMU_REG_XG_OFFSET_H, offset >> 8);
    spiHalImuWriteReg(IMU_REG_XG_OFFSET_L, offset);
}
/*
static int16_t getYGyroOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = spiHalImuReadReg(IMU_REG_YG_OFFSET_H);
    buffer[1] = spiHalImuReadReg(IMU_REG_YG_OFFSET_L);

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
*/
static void setYGyroOffset(int16_t offset) {
    spiHalImuWriteReg(IMU_REG_YG_OFFSET_H, offset >> 8);
    spiHalImuWriteReg(IMU_REG_YG_OFFSET_L, offset);
}
/*
static int16_t getZGyroOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = spiHalImuReadReg(IMU_REG_ZG_OFFSET_H);
    buffer[1] = spiHalImuReadReg(IMU_REG_ZG_OFFSET_L);

    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
*/
static void setZGyroOffset(int16_t offset)
{
    spiHalImuWriteReg(IMU_REG_ZG_OFFSET_H, offset >> 8);
    spiHalImuWriteReg(IMU_REG_ZG_OFFSET_L, offset);
}

static void getImuValues(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint16_t mpuData16[7], temp;
    uint8_t *mpuData = (uint8_t *) mpuData16;
    imuReadBuff(IMU_REG_ACCEL_XOUT_H, mpuData, sizeof(mpuData));

    temp = mpuData[0];
    mpuData[0] = mpuData[1];
    mpuData[1] = temp;
    temp = mpuData[2];
    mpuData[2] = mpuData[3];
    mpuData[3] = temp;
    temp = mpuData[4];
    mpuData[4] = mpuData[5];
    mpuData[5] = temp;

    temp = mpuData[8];
    mpuData[7] = temp;
    mpuData[6] = mpuData[9];
    temp = mpuData[10];
    mpuData[9] = temp;
    mpuData[8] = mpuData[11];
    temp = mpuData[12];
    mpuData[11] = temp;
    mpuData[10] = mpuData[13];

    *ax = *((uint16_t *) &mpuData[0]);
    *ay = *((uint16_t *) &mpuData[2]);
    *az = *((uint16_t *) &mpuData[4]);

    *gx = *((uint16_t *) &mpuData[6]);
    *gy = *((uint16_t *) &mpuData[8]);
    *gz = *((uint16_t *) &mpuData[10]);
}

uint32_t absI(int32_t val)
{
    return val < 0 ? -val : val;
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

void calibrationProcess(int32_t *gxOffset, int32_t *gyOffset, int32_t *gzOffset, int32_t *axOffset, int32_t *ayOffset, int32_t *azOffset)
{
    int32_t meanAx, meanAy, meanAz, meanGx, meanGy, meanGz;
    bool isXaReady = false, isYaReady = false, isZaReady = false;
    getMeanValues(&meanAx, &meanAy, &meanAz, &meanGx, &meanGy, &meanGz);
    *gxOffset = -(meanGx >> 2);
    *gyOffset = -(meanGy >> 2);
    *gzOffset = -(meanGz >> 2);

    for (;;) {
        setXGyroOffset(*gxOffset);
        setYGyroOffset(*gyOffset);
        setZGyroOffset(*gzOffset);

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

        if (isXaReady && isYaReady && isZaReady)
            break;
    }
}

void readMagData(int16_t * destination)
{
    imuReadBuff(IMU_REG_EXT_SENS_DATA_00, (uint8_t *) destination, 6);
}

#define SECTOR_7_START  0x08060000
#define SECTOR_NUM      FLASH_Sector_7
#define WATERMARK       0xC0FEBABE


typedef struct MagCalibration {
    float magnetometerBias[3];
    float magnetometerSensitivity[3];
    uint32_t watermark;
} MagCalibration;

MagCalibration magCalibration, magOld;

void sensorSystemCalibrate(float magnetometerBias[3], float magnetometerSensitivity[3], bool isBiasUpdateNeeded)
{
    int32_t gxOffset = 48, gyOffset = -9, gzOffset = 30;
    int32_t axBias = 0, ayBias = 0, azBias = 0;

    if (isBiasUpdateNeeded) {
        calibrationProcess(&gxOffset, &gyOffset, &gzOffset, &axBias, &ayBias, &azBias);
    } else {
        setXGyroOffset(gxOffset);
        setYGyroOffset(gyOffset);
        setZGyroOffset(gzOffset);
    }

    uint32_t cntr1 = 0, cntr2 = 0, cntr3 = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

    sample_count = 6;
    for(cntr1 = 0; cntr1 < sample_count; cntr1++) {
        readMagData(mag_temp);
        for (cntr2 = 0; cntr2 < 3; cntr2++) {
            if(mag_temp[cntr2] > mag_max[cntr2]) {
                 mag_max[cntr2] = mag_temp[cntr2];
            }
            if(mag_temp[cntr2] < mag_min[cntr2]) {
                mag_min[cntr2] = mag_temp[cntr2];
            }
        }
        for (cntr3 = 0; cntr3 < 3; cntr3++);
    }

    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

    const float magResolution = 4912.0f / 32760.0f;

    magnetometerBias[0] = (float) mag_bias[0] * magResolution * fMagCalibrationX;
    magnetometerBias[1] = (float) mag_bias[1] * magResolution * fMagCalibrationY;
    magnetometerBias[2] = (float) mag_bias[2] * magResolution * fMagCalibrationZ;

    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

    const float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0f;

    magnetometerSensitivity[0] = avg_rad / ((float) mag_scale[0]);
    magnetometerSensitivity[1] = avg_rad / ((float) mag_scale[1]);
    magnetometerSensitivity[2] = avg_rad / ((float) mag_scale[2]);

    magCalibration.magnetometerBias[0] = magnetometerBias[0];
    magCalibration.magnetometerBias[1] = magnetometerBias[1];
    magCalibration.magnetometerBias[2] = magnetometerBias[2];

    magCalibration.magnetometerSensitivity[0] = magnetometerSensitivity[0];
    magCalibration.magnetometerSensitivity[1] = magnetometerSensitivity[1];
    magCalibration.magnetometerSensitivity[2] = magnetometerSensitivity[2];

    magCalibration.watermark = WATERMARK;
    magOld = *((MagCalibration *) SECTOR_7_START);
    if ((((MagCalibration *) SECTOR_7_START))->watermark != WATERMARK) {
        FLASH_Unlock();
        uint32_t cntr;
        for (cntr = 0; cntr < 1000; cntr++);

        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGPERR | FLASH_FLAG_WRPERR);
        FLASH_WaitForLastOperation();
        FLASH_EraseSector(SECTOR_NUM, VoltageRange_3);
        FLASH_WaitForLastOperation();
        for (cntr = 0; cntr < sizeof(magCalibration); cntr++) {
            while (FLASH_ProgramByte(SECTOR_7_START + cntr, ((uint8_t *) &magCalibration)[cntr]) != FLASH_COMPLETE) {

            }
            FLASH_WaitForLastOperation();
        }
        FLASH_Lock();
    }
    magnetometerBias[0] = magOld.magnetometerBias[0];
    magnetometerBias[1] = magOld.magnetometerBias[1];
    magnetometerBias[2] = magOld.magnetometerBias[2];
    magnetometerSensitivity[0] = magOld.magnetometerSensitivity[0];
    magnetometerSensitivity[1] = magOld.magnetometerSensitivity[1];
    magnetometerSensitivity[2] = magOld.magnetometerSensitivity[2];
}

void clearFlash(void) {
    FLASH_Unlock();
    FLASH_EraseSector(SECTOR_NUM, VoltageRange_3);
    FLASH_Lock();
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

#endif
