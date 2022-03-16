#ifndef USART_H
#define USART_H

#include "stm32f030x6.h"

#define PACKET_SIZE 10
#define BUFFER_SIZE 2 * PACKET_SIZE

void UART_Init(void);
uint8_t UART_RXNE(void);
void UART_ReadBuffer(uint8_t *bytes);
uint8_t UART_ReadBytes(uint8_t *bytes, uint8_t size);
void UART_WriteBytes(const uint8_t *bytes, uint8_t size);

#endif // USART_H
