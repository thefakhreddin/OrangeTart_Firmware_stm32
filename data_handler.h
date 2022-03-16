#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include "stm32f030x6.h"

/* Programming */
extern uint8_t g_program_len;
extern uint8_t is_executing;
extern uint8_t g_executing_index;
extern uint32_t g_wait_start;
extern uint32_t g_wait_duration;

void Data_Handler_Handle(uint8_t *data);
void Execute(uint16_t index);
void Data_Handler_Stop(void);

#endif // DATA_HANDLER_H
