#ifndef MOTOR_SPIN_TEST_H
#define MOTOR_SPIN_TEST_H

#include "main.h"

void MotorSpinTest_Run(UART_HandleTypeDef *debug_uart,
                       UART_HandleTypeDef *motor_uart,
                       GPIO_TypeDef *dir_port,
                       uint16_t dir_pin);

#endif
