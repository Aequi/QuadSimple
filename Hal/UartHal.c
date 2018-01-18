#include "UartHal.h"
#include "HalCommon.h"
#include "stm32f4xx_conf.h"
#include <stddef.h>

#define UART_BT_TX_PIN      GPIO_Pin_2
#define UART_BT_TX_PIN_SRC  GPIO_PinSource2
#define UART_BT_TX_PORT     GPIOA

#define UART_BT_RX_PIN      GPIO_Pin_3
#define UART_BT_RX_PIN_SRC  GPIO_PinSource3
#define UART_BT_RX_PORT     GPIOA

#define UART_BT_PINS_AF     GPIO_AF_USART2

#define UART_BT_PORT        GPIOA
#define UART_HW_UNIT        USART2
#define UART_DMA_RX_HW_UNIT DMA1_Stream5
#define UART_DMA_TX_HW_UNIT DMA1_Stream6
#define UART_DMA_CHANNEL    DMA_Channel_4
#define UART_DMA_TX_IT_FLAG DMA_IT_TCIF6
#define UART_DMA_TX_IRQ     DMA1_Stream6_IRQHandler
#define UART_DMA_TX_IRQn    DMA1_Stream6_IRQn


uint8_t uartCmdInBuffer[UART_CMD_BUFF_SIZE];
static volatile bool uartBtDmaTxActive = false;
static void (*uartBtDmaTxCompleteCb)() = NULL;

void uartBtInit(uint32_t baudRate)
{
    GPIO_InitTypeDef gpioInitStructure = {.GPIO_Pin = UART_BT_TX_PIN | UART_BT_RX_PIN,
                                          .GPIO_Mode = GPIO_Mode_AF,
                                          .GPIO_Speed = GPIO_Speed_50MHz,
                                          .GPIO_OType = GPIO_OType_PP,
                                          .GPIO_PuPd = GPIO_PuPd_UP};

    GPIO_Init(UART_BT_PORT, &gpioInitStructure);

    GPIO_PinAFConfig(UART_BT_PORT, UART_BT_TX_PIN_SRC, UART_BT_PINS_AF);
    GPIO_PinAFConfig(UART_BT_PORT, UART_BT_RX_PIN_SRC, UART_BT_PINS_AF);

    USART_InitTypeDef usartInitStructure = {.USART_BaudRate = baudRate,
                                            .USART_WordLength = USART_WordLength_8b,
                                            .USART_StopBits = USART_StopBits_1,
                                            .USART_Parity = USART_Parity_No,
                                            .USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
                                            .USART_HardwareFlowControl = USART_HardwareFlowControl_None};

    USART_Init(UART_HW_UNIT, &usartInitStructure);
    USART_Cmd(UART_HW_UNIT, ENABLE);
    USART_DMACmd(UART_HW_UNIT, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

    DMA_InitTypeDef dmaInitStructure;

    dmaInitStructure.DMA_BufferSize = sizeof(uartCmdInBuffer);
    dmaInitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dmaInitStructure.DMA_Memory0BaseAddr = (uint32_t) uartCmdInBuffer;
    dmaInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStructure.DMA_Mode = DMA_Mode_Circular;
    dmaInitStructure.DMA_PeripheralBaseAddr = (uint32_t) &UART_HW_UNIT->DR;
    dmaInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure.DMA_Priority = DMA_Priority_Medium;
    dmaInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dmaInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dmaInitStructure.DMA_Channel = UART_DMA_CHANNEL;
    dmaInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_DoubleBufferModeCmd(UART_DMA_RX_HW_UNIT, DISABLE);
    DMA_Init(UART_DMA_RX_HW_UNIT, &dmaInitStructure);
}

void UART_DMA_TX_IRQ(void)
{
    DMA_ClearITPendingBit(UART_DMA_TX_HW_UNIT, UART_DMA_TX_IT_FLAG);
    DMA_Cmd(UART_DMA_TX_HW_UNIT, DISABLE);
    uartBtDmaTxActive = false;
    if (uartBtDmaTxCompleteCb)
        uartBtDmaTxCompleteCb();
}

void uartBtDmaTx(uint8_t *dataBuffer, uint32_t bufferSize, void (*txCompleteCb)())
{
    if (uartBtDmaTxActive)
        return;

    if (txCompleteCb)
        uartBtDmaTxCompleteCb = txCompleteCb;

    DMA_InitTypeDef dmaInitStructure;

    dmaInitStructure.DMA_BufferSize = bufferSize;
    dmaInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dmaInitStructure.DMA_Memory0BaseAddr = (uint32_t) dataBuffer;
    dmaInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dmaInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dmaInitStructure.DMA_Mode = DMA_Mode_Normal;
    dmaInitStructure.DMA_PeripheralBaseAddr = (uint32_t) &UART_HW_UNIT->DR;
    dmaInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dmaInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dmaInitStructure.DMA_Priority = DMA_Priority_Medium;
    dmaInitStructure.DMA_Channel = UART_DMA_CHANNEL;
    dmaInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dmaInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    dmaInitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dmaInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_DoubleBufferModeCmd(UART_DMA_TX_HW_UNIT, DISABLE);
    DMA_Init(UART_DMA_TX_HW_UNIT, &dmaInitStructure);
    NVIC_EnableIRQ(UART_DMA_TX_IRQn);
    DMA_ITConfig(UART_DMA_TX_HW_UNIT, DMA_IT_TC, ENABLE);
    uartBtDmaTxActive = true;
    DMA_Cmd(UART_DMA_TX_HW_UNIT, ENABLE);
}

uint32_t uartBtGetCurrentRxBuffIdx(void)
{
    return UART_CMD_BUFF_SIZE - UART_DMA_RX_HW_UNIT->NDTR;
}

void uartBtDmaStartStopRx(bool isStart)
{
    DMA_Cmd(UART_DMA_RX_HW_UNIT, isStart ? ENABLE : DISABLE);
}
