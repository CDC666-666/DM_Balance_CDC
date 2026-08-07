#ifndef GAMEPAD_TASK_H
#define GAMEPAD_TASK_H

#include <stdint.h>

extern volatile uint32_t gamepad_uart_error_count;
extern volatile uint32_t gamepad_uart_overflow_count;

void Gamepad_Task(void);

#endif
