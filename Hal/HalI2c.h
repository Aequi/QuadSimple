#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include <stdbool.h>
#include <stdint.h>

typedef void (*HalI2cDmaReadCallback)(void);
typedef void (*HalIntPinEventCallback)(void);

void halI2cInit(HalI2cDmaReadCallback halI2cDmaReadCb, HalIntPinEventCallback halIntPinEventCb);
void halI2cWrite(uint8_t chipAddress, uint8_t registerAddress, uint8_t data);
uint8_t halI2cRead(uint8_t chipAddress, uint8_t registerAddress);
void halI2cReadBuf(uint8_t chipAddress, uint8_t registerAddress, uint8_t data[], uint32_t dataLength);
void halI2cReadWithDma(uint8_t chipAddress, uint8_t startRegisterAddress, uint32_t length);
uint8_t *halI2cGetReadBuffer(uint32_t *bufferSize);

#endif
