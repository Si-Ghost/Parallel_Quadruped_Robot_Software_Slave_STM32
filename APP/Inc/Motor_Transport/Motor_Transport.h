#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_TRANSPORT_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_TRANSPORT_H

#include "main.h"

/*
 * Four-UART RS485 motor transport.
 *
 * This module owns UART2/UART8/UART7/UART5 while running.  Call Stop before
 * any blocking motor transaction (for example Leg_Control_Handshake), then
 * call Start afterwards to re-arm all circular RX DMA rings.
 */
void Motor_Transport_Start(void);
void Motor_Transport_Stop(void);
void Motor_Transport_Tick(void);
void Motor_Transport_Service(void);

/* Return non-zero only when the UART belongs to this transport. */
uint8_t Motor_Transport_HandleTxComplete(UART_HandleTypeDef *huart);
uint8_t Motor_Transport_HandleError(UART_HandleTypeDef *huart);

#endif
