#ifndef __UART_HAL_H__
#define __UART_HAL_H__

#include <stdbool.h>
#include <stdint.h>

#define UART_CMD_BUFF_SIZE  128
#define UART_CMD_BUFF_MASK  0x7F

extern uint8_t uartCmdInBuffer[UART_CMD_BUFF_SIZE];

void uartBtInit(uint32_t baudRate);
void uartBtDmaTx(uint8_t *dataBuffer, uint32_t bufferSize, void (*txCompleteCb)());
uint32_t uartBtGetCurrentRxBuffIdx(void);
void uartBtDmaStartStopRx(bool isStart);

#endif
