#include "SensorSystem.h"
#include "HalI2c.h"

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

static int8_t getXFineGain(void)
{
    return i2cRead(MPU_6050_ADDRESS, MPU6050_RA_X_FINE_GAIN);
}

static void setXFineGain(int8_t gain)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_X_FINE_GAIN, gain);
}

static int8_t getYFineGain(void)
{
    return i2cRead(MPU_6050_ADDRESS, MPU6050_RA_Y_FINE_GAIN);
}

void setYFineGain(int8_t gain)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_Y_FINE_GAIN, gain);
}

static int8_t getZFineGain(void)
{
    return i2cRead(MPU_6050_ADDRESS, MPU6050_RA_Z_FINE_GAIN);
}

static void setZFineGain(int8_t gain)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_Z_FINE_GAIN, gain);
}

static int16_t getXAccelOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H + 1);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}

static void setXAccelOffset(int16_t offset)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H + 1, offset);
}

static int16_t getYAccelOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

static void setYAccelOffset(int16_t offset)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H + 1, offset);
}

static int16_t getZAccelOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

static void setZAccelOffset(int16_t offset)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H + 1, offset);
}

static int16_t getXGyroOffset(void)
{
    uint8_t buffer[2];
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

static void setXGyroOffset(int16_t offset)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH + 1, offset);
}

static int16_t getYGyroOffset()
{
    uint8_t buffer[2];
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

static void setYGyroOffset(int16_t offset)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH + 1, offset);
}

static int16_t getZGyroOffset()
{
    uint8_t buffer[2];
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

static void setZGyroOffset(int16_t offset)
{
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH + 1, offset);
}

SensorSystemUpdateCallback sensorSystemUpdateCallback;

static void onI2cDmaReadyEvent(void)
{
    if (sensorSystemUpdateCallback != NULL) {
        sensorSystemUpdateCallback();
    }
}

static void onIntEvent(void)
{

}

static void setupImu(uint8_t sampleRateDiv, uint8_t dlpf, uint8_t gyroFsel, uint8_t accFsel)
{
    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_PWR_MGMT_1, 0x01);

    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_SMPLRT_DIV, sampleRateDiv);
    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_CONFIG, dlpf);
    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_GYRO_CONFIG, gyroFsel << 3);
    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_ACCEL_CONFIG, accFsel << 3);

    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_INT_PIN_CFG, 0x10);
}

void setupImuInt(bool isEnabled)
{
    i2cWrite(MPU_6050_ADDRESS, REG_ADDR_INT_ENABLE, isEnabled ? 0x01 : 0x00);
    i2cRead(MPU_6050_ADDRESS, REG_ADDR_INT_STATUS);
}

#if 0

static uint32_t absI(int32_t value)
{
    return value < 0 ? -value : value;
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

  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(8192-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  uint8_t xorSt = 0;
  while (1){
    int ready=0;

    setXAccelOffset(ax_offset);
    setYAccelOffset(ay_offset);
    setZAccelOffset(az_offset);

    setXGyroOffset(gx_offset);
    setYGyroOffset(gy_offset);
    setZGyroOffset(gz_offset);

    meansensors();
    if (xorSt)
        LCDStr(0, 15, "...");
    else
        LCDStr(0, 15, "   ");
    xorSt ^= 1;

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(8192-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(8192-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }

       uint32_t cnt;
  if (state==0){
    meansensors();
    state++;

    for (cnt=0;cnt<10000000;cnt++);
  }

  if (state==1) {
    calibration();
    state++;
    for (cnt=0;cnt<10000000;cnt++);
  }

  if (state==2) {
    meansensors();
    LCDStr(0, 0, "FINISHED!");
    LCDStr(0, 1, "Readings:");
    char val[10];
    itoa(mean_ax, val);
    LCDStr(0, 2, val);
    itoa(mean_ay, val);
    LCDStr(0, 3, val);
    itoa(mean_az, val);
    LCDStr(0, 4, val);
    itoa(mean_gx, val);
    LCDStr(0, 5, val);
    itoa(mean_gy, val);
    LCDStr(0, 6, val);
    itoa(mean_gz, val);
    LCDStr(0, 7, val);
    LCDStr(0, 8, "Offsets:");
    itoa(ax_offset, val);
    LCDStr(0, 9, val);
    itoa(ay_offset, val);
    LCDStr(0, 10, val);
    itoa(az_offset, val);
    LCDStr(0, 11, val);
    itoa(gx_offset, val);
    LCDStr(0, 12, val);
    itoa(gy_offset, val);
    LCDStr(0, 13, val);
    itoa(gz_offset, val);
    LCDStr(0, 14, val);
    while (1);
  }

}

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
}

#define M_PI_F  3.1415926535897932384626433832795f
static float currentPitch, currentRoll, currentYaw;

static void eulerFromQuaternion(Quaternion *quaternion)
{
    float gx = 2.0f * (quaternion->x * quaternion->z - quaternion->w * quaternion->y);
    float gy = 2.0f * (quaternion->w * quaternion->x + quaternion->y * quaternion->z);
    float gz = quaternion->w * quaternion->w - quaternion->x * quaternion->x - quaternion->y * quaternion->y + quaternion->z * quaternion->z;

    if (gx > 1.0f) {
        gx = 1.0f;
    }
    if (gx < -1.0f) {
        gx = -1.0f;
    }

  currentYaw = atan2f(2.0f * (quaternion->w * quaternion->z + quaternion->x * quaternion->y),
                quaternion->w * quaternion->w + quaternion->x * quaternion->x - quaternion->y * quaternion->y - quaternion->z * quaternion->z) * 180.0f / M_PI_F;
  currentPitch = asinf(gx) * 180.0f / M_PI_F;
  currentRoll = atan2f(gy, gz) * 180.0f / M_PI_F;
}

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

void meansensors();

int buffersize = 1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

void getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t mpuData[14], temp;
    i2cReadBuf(MPU_6050_ADDRESS, 0x3B, mpuData, sizeof(mpuData));
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

#endif

void sensorSystemInit(SensorSystemUpdateCallback sensorSystemUpdateCb, uint32_t updateRate)
{
    sensorSystemUpdateCallback = sensorSystemUpdateCb;
    halI2cInit(onI2cDmaReadyEvent, onIntEvent);
    setupMpu6050(1, 2, 3, 1);
    setupCalibration(3, 1, 2, 1);

    #define OF_FILTER_COEF_ALPHA  0.0002f
    #define OF_FILTER_COEF_BETA   0.0002f
    #define OF_FILTER_COEF_GAMMA  0.00000002f
    #define OF_FILTER_COEF_THETA  0.0002f
    orientationFilterInitialize(&orientationFilterCtx, 1.0f / (float) SENSOR_SYSTEM_UPDATE_RATE, OF_FILTER_COEF_ALPHA, OF_FILTER_COEF_ALPHA, OF_FILTER_COEF_GAMMA, OF_FILTER_COEF_THETA);
}

void sensorSystemCalibrate(void)
{

}

void sensorSystemGetCurrentOrientation(FlightControllerOrientationEstimate *orientationEstimate)
{

}

void sensorSystemStartUpdateEvent(void)
{

}
