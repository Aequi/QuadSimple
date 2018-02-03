#include "HalI2c.h"
#include "stm32f0xx_conf.h"

#include <stddef.h>

#define HAL_I2C_IMU_SDA_PIN                             GPIO_Pin_10
#define HAL_I2C_IMU_SDA_PIN_SOURCE                      GPIO_PinSource10

#define HAL_I2C_IMU_SCL_PIN                             GPIO_Pin_9
#define HAL_I2C_IMU_SCL_PIN_SOURCE                      GPIO_PinSource9

#define HAL_INT_IMU_PIN                                 GPIO_Pin_1
#define HAL_INT_IMU_PIN_SOURCE                          GPIO_PinSource1

#define HAL_I2C_IMU_PORT                                GPIOA
#define HAL_INT_IMU_PORT                                GPIOB

#define I2C_PERIPH_HWUNIT                               I2C1

#define I2C_PERIPH_OWN_ADDRESS                          1u

#define I2C_PERIPH_IMU_ADDRESS                          (0x68 << 1)

static void i2cInit(void)
{
    GPIO_InitTypeDef gpioInitStructure = {
        .GPIO_Pin = HAL_I2C_IMU_SDA_PIN | HAL_I2C_IMU_SCL_PIN,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_PuPd = GPIO_PuPd_UP
    };

    GPIO_Init(HAL_I2C_IMU_PORT, &gpioInitStructure);

    gpioInitStructure.GPIO_Mode = GPIO_Mode_IN;
    gpioInitStructure.GPIO_Pin = HAL_INT_IMU_PIN;
    gpioInitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(HAL_INT_IMU_PORT, &gpioInitStructure);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

    EXTI_InitTypeDef extiInitStructure = {
        .EXTI_Line = EXTI_Line1,
        .EXTI_Mode = EXTI_Mode_Interrupt,
        .EXTI_Trigger = EXTI_Trigger_Falling,
        .EXTI_LineCmd = ENABLE,
    };

    EXTI_Init(&extiInitStructure);
    NVIC_EnableIRQ(EXTI0_1_IRQn);

    GPIO_PinAFConfig(HAL_I2C_IMU_PORT, HAL_I2C_IMU_SDA_PIN_SOURCE, GPIO_AF_4);
    GPIO_PinAFConfig(HAL_I2C_IMU_PORT, HAL_I2C_IMU_SCL_PIN_SOURCE, GPIO_AF_4);

    I2C_InitTypeDef i2cInitStructure;
    I2C_StructInit(&i2cInitStructure);
    i2cInitStructure.I2C_Mode = I2C_Mode_I2C;
    i2cInitStructure.I2C_Timing = 0x00000102;
    i2cInitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInitStructure.I2C_OwnAddress1 = I2C_PERIPH_OWN_ADDRESS;
    i2cInitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_Init(I2C_PERIPH_HWUNIT, &i2cInitStructure);
    I2C_AcknowledgeConfig(I2C_PERIPH_HWUNIT, ENABLE);
    I2C_StretchClockCmd(I2C_PERIPH_HWUNIT, ENABLE);
    I2C_Cmd(I2C_PERIPH_HWUNIT, ENABLE);
}

static void i2cWrite(uint8_t chipAddress, uint8_t registerAddress, uint8_t data)
{
    while (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_BUSY));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C_PERIPH_HWUNIT, registerAddress);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

	I2C_SendData(I2C_PERIPH_HWUNIT, data);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 0, I2C_AutoEnd_Mode,  I2C_Generate_Stop);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);
}

static uint8_t i2cRead(uint8_t chipAddress, uint8_t registerAddress)
{
    while (I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_BUSY));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	I2C_SendData(I2C_PERIPH_HWUNIT, registerAddress);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_TXE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Read);
    while(I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_ISR_TXIS) == RESET);

	uint8_t data = I2C_ReceiveData(I2C_PERIPH_HWUNIT);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_RXNE));

    I2C_TransferHandling(I2C_PERIPH_HWUNIT, chipAddress, 0, I2C_AutoEnd_Mode,  I2C_Generate_Stop);
    while(!I2C_GetFlagStatus(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH_HWUNIT, I2C_FLAG_STOPF);

    return data;
}


void halI2cImuInit(void)
{
    i2cInit();
    uint8_t id = i2cRead(I2C_PERIPH_IMU_ADDRESS, 0x75);
    id = i2cRead(I2C_PERIPH_IMU_ADDRESS, 0x75);
}

void EXTI0_1_IRQHandler(void)
{

}
#if 0
#include "stm32f4xx_conf.h"
#include "driver.h"
#include "fatext.h"
#include "ff.h"

#include "BlueTooth.h"
#include "i2cHAL.h"

#define MPU_6050_ADDRESS    (0x68 << 1)

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
#define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18

uint8_t buffer[2];

// X_FINE_GAIN register

int8_t getXFineGain() {
    return i2cRead(MPU_6050_ADDRESS, MPU6050_RA_X_FINE_GAIN);
}
void setXFineGain(int8_t gain) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register

int8_t getYFineGain() {
    return i2cRead(MPU_6050_ADDRESS, MPU6050_RA_Y_FINE_GAIN);
}
void setYFineGain(int8_t gain) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register

int8_t getZFineGain() {

    return i2cRead(MPU_6050_ADDRESS, MPU6050_RA_Z_FINE_GAIN);
}
void setZFineGain(int8_t gain) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers

int16_t getXAccelOffset() {
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H + 1);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void setXAccelOffset(int16_t offset) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XA_OFFS_H + 1, offset);
}

// YA_OFFS_* register

int16_t getYAccelOffset() {
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setYAccelOffset(int16_t offset) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YA_OFFS_H + 1, offset);
}

// ZA_OFFS_* register

int16_t getZAccelOffset() {
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setZAccelOffset(int16_t offset) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZA_OFFS_H + 1, offset);
}

// XG_OFFS_USR* registers

int16_t getXGyroOffset() {
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setXGyroOffset(int16_t offset) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_XG_OFFS_USRH + 1, offset);
}

// YG_OFFS_USR* register

int16_t getYGyroOffset() {
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setYGyroOffset(int16_t offset) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_YG_OFFS_USRH + 1, offset);
}

// ZG_OFFS_USR* register

int16_t getZGyroOffset() {
    buffer[0] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH);
    buffer[1] = i2cRead(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH + 1);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void setZGyroOffset(int16_t offset) {
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, offset >> 8);
    i2cWrite(MPU_6050_ADDRESS, MPU6050_RA_ZG_OFFS_USRH + 1, offset);
}

void meansensors();
int lenStr (char s[])
{
    int i = 0;
    while (*s) {
        i++;
        s++;
    }
    return i;
}

 void reverse(char s[])
 {
     int i, j;
     char c;

     for (i = 0, j = lenStr(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
 }


 void itoa(int n, char s[])
 {
     int i, sign;

     if ((sign = n) < 0)  /* записываем знак */
         n = -n;          /* делаем n положительным числом */
     i = 0;
     do {       /* генерируем цифры в обратном порядке */
         s[i++] = n % 10 + '0';   /* берем следующую цифру */
     } while ((n /= 10) > 0);     /* удаляем */
     if (sign < 0)
         s[i++] = '-';
     s[i] = 0;
     reverse(s);
 }


///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)


int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setupCalibration(uint8_t sampleRateDiv, uint8_t dlpf, uint8_t gyroFsel, uint8_t accFsel) {
    i2cWrite(MPU_6050_ADDRESS, 0x6B, 0x01);
    i2cWrite(MPU_6050_ADDRESS, 0x19, sampleRateDiv);
    i2cWrite(MPU_6050_ADDRESS, 0x1A, dlpf);
    i2cWrite(MPU_6050_ADDRESS, 0x1B, gyroFsel << 3);
    i2cWrite(MPU_6050_ADDRESS, 0x1C, accFsel << 3);

    i2cWrite(MPU_6050_ADDRESS, 0x37, 0b1000);
    i2cRead(MPU_6050_ADDRESS, 0x3A);
    i2cWrite(MPU_6050_ADDRESS, 0x38, 0x00);

    setXAccelOffset(0);
    setYAccelOffset(0);
    setZAccelOffset(0);
    setXGyroOffset(0);
    setYGyroOffset(0);
    setZGyroOffset(0);

}

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

uint32_t abs(int32_t val)
{
return val < 0 ? -val : val;
}

void calibration(){
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
}



///////////////////////////////////   LOOP   ////////////////////////////////////
void calibrationLoop() {
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

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    uint32_t cntr;
    for(cntr = 0; cntr < 100; cntr++); //Needed so we don't get repeated measures
  }
}


FIL test;
FIL wr;
volatile uint8_t btTrStatus = 0, mpuDataReady = 0;


void setupMpu6050(uint8_t sampleRateDiv, uint8_t dlpf, uint8_t gyroFsel, uint8_t accFsel)
{
    EXTI_InitTypeDef extiInitStruct;
    GPIO_InitTypeDef GPIOInitStruct;
    GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIOInitStruct.GPIO_OType = GPIO_OType_PP;
    GPIOInitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIOInitStruct.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOB, &GPIOInitStruct);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource9);
    extiInitStruct.EXTI_Line = EXTI_Line9;
    extiInitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    extiInitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    extiInitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&extiInitStruct);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    i2cWrite(MPU_6050_ADDRESS, 0x6B, 0x01);

    i2cWrite(MPU_6050_ADDRESS, 0x19, sampleRateDiv);
    i2cWrite(MPU_6050_ADDRESS, 0x1A, dlpf);
    i2cWrite(MPU_6050_ADDRESS, 0x1B, gyroFsel << 3);
    i2cWrite(MPU_6050_ADDRESS, 0x1C, accFsel << 3);

    setXAccelOffset(-171);
    setYAccelOffset(2153);
    setZAccelOffset(1432);
    setXGyroOffset(72);
    setYGyroOffset(17);
    setZGyroOffset(-5);

    i2cWrite(MPU_6050_ADDRESS, 0x37, 0x10);
    i2cRead(MPU_6050_ADDRESS, 0x3A);
    i2cWrite(MPU_6050_ADDRESS, 0x38, 0x01);


}

uint8_t mpu6050Data[14];
char vali[10];
uint32_t cntrFr = 0;
volatile bool isItBad = 0;
void EXTI9_5_IRQHandler()
{
    EXTI_ClearITPendingBit(EXTI_Line9);
    EXTI_ClearFlag(EXTI_Line9);

    i2cReadBuf(MPU_6050_ADDRESS, 0x3B, mpu6050Data, sizeof(mpu6050Data));
    mpu6050Data[6] = mpu6050Data[8];
    mpu6050Data[7] = mpu6050Data[9];
    mpu6050Data[8] = mpu6050Data[10];
    mpu6050Data[9] = mpu6050Data[11];
    mpu6050Data[10] = mpu6050Data[12];
    mpu6050Data[11] = mpu6050Data[13];
    if (btTrStatus)
        blueToothSend(mpu6050Data, 12);
    /*
    cntrFr++;
    if (cntrFr % 250 == 0) {
        ax = ((uint16_t) mpu6050Data[0] << 8) | mpu6050Data[1];
        ay = ((uint16_t) mpu6050Data[2] << 8) | mpu6050Data[3];
        az = ((uint16_t) mpu6050Data[4] << 8) | mpu6050Data[5];

        gx = ((uint16_t) mpu6050Data[6] << 8) | mpu6050Data[7];
        gy = ((uint16_t) mpu6050Data[8] << 8) | mpu6050Data[9];
        gz = ((uint16_t) mpu6050Data[10] << 8) | mpu6050Data[11];

        itoa(ax, vali);
        LCDStr(0, 1, "       ");
        LCDStr(0, 1, vali);
        itoa(ay, vali);
        LCDStr(0, 3, "       ");
        LCDStr(0, 3, vali);
        itoa(az, vali);
        LCDStr(0, 5, "       ");
        LCDStr(0, 5, vali);
        itoa(gx, vali);
        LCDStr(0, 7, "       ");
        LCDStr(0, 7, vali);
        itoa(gy, vali);
        LCDStr(0, 9, "       ");
        LCDStr(0, 9, vali);
        itoa(gz, vali);
        LCDStr(0, 11, "       ");
        LCDStr(0, 11, vali);
    }*/
    mpuDataReady = 1;
}

void btByteReady(uint8_t data);

char str[10];
int main(void)
{
	GPIO_InitTypeDef GPIOInitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	/*Enable shared resources clock*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIOInitStruct.GPIO_OType = GPIO_OType_PP;
    GPIOInitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIOInitStruct);
    GPIOA->ODR = 0b0000;
uint32_t cnt;
for (cnt=0;cnt<10000000;cnt++);
    GPIOInitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIOInitStruct.GPIO_OType = GPIO_OType_PP;
    GPIOInitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;
    GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIOInitStruct);
    GPIOB->BSRRL = 0b100000000000000;
    GPIOB->BSRRH = 0b1000000000000;

    for (cnt=0;cnt<5000000;cnt++);

    GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIOInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIOInitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIOInitStruct);

    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0)) {
        GPIOB->BSRRL = 0b10000000000000;
        LCDInit();
        i2cInit();
        //LCDSet();
        //GPIOB->BSRRL = 0b100000000000000;
        //for(;;);
        //blueToothHalInit(9600);
        //blueToothSend("AT+UART=1382400,1,0\r\n", 21);
        blueToothHalInit(1382400);
        setupMpu6050(1, 2, 3, 1);
        GPIOB->BSRRH = 0b1000000000000;

        /* Output settings here */
        LCDStr(0, 0, "Name: MotionTracker");
        LCDStr(0, 2, "Pass: 1234");
        LCDStr(0, 4, "Sampling Rate: 500");
        LCDStr(0, 6, "Ac scale: +/-4 g");
        LCDStr(0, 8, "Gy scale: +/-2000 \x09/s");
        LCDStr(0, 10, "16 bit signed");
        LCDStr(0, 12, "ACC-x,y,z, GYRO-x,y,z");
        LCDStr(0, 14, "Host command:");
        for(;;){
            /*
        GPIOB->BSRRL = 0b100000000000000;

        for (cnt=0;cnt<100000;cnt++);
        GPIOB->BSRRH = 0b100000000000000;
    //    blueToothSend((uint8_t *) "TEST!!!\r\n", 9);
        for (cnt=0;cnt<100000;cnt++);
            */

        }
    //	fsStart(0);
    //	FIL test;
    //	f_open(&test, "num.txt", FA_OPEN_EXISTING | FA_READ);
    //	UINT bw;
    //	uint8_t num = 0;
    //	FRESULT res;
    //	res = f_read(&test, &num, 1, &bw);
    //	f_close(&test);
     //   fsStop();
        for(;;);
    } else {
        LCDInit();
        i2cInit();
        setupCalibration(3, 1, 2, 1);
        calibrationLoop();
        for(;;);
    }
    return 0;
}

uint8_t xChr = 0, yChr = 15;
void btByteReady(uint8_t data)
{
    if (data == '+')
        btTrStatus = 1;
    else if (data == '-')
        btTrStatus = 0;
    else {
//        LCDChrXY(xChr, yChr, data);
//        xChr++;
//        if (xChr == 22){
//            xChr = 0;
//            yChr++;
//            if(yChr == 16){
//                yChr = 15;
//                LCDStr(0, 15, "                     ");
//            }
//        }
    }
}

#endif
