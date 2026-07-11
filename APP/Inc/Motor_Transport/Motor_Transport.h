#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_TRANSPORT_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_TRANSPORT_H

#include "main.h"

/*
 * Owns UART2/UART8/UART7/UART5 only while running.
 * Init must be called before Stop/Start because the DMA state lives in a
 * NOLOAD RAM_D2 section.  Stop restores normal RX DMA for blocking handshakes;
 * Start switches RX DMA to circular and arms all four permanent rings.
 */
HAL_StatusTypeDef Motor_Transport_Init(void);
HAL_StatusTypeDef Motor_Transport_Start(void);
HAL_StatusTypeDef Motor_Transport_Stop(void);
void Motor_Transport_Tick(void);
void Motor_Transport_Service(void);

/* Return non-zero only when the UART belongs to this transport. */
uint8_t Motor_Transport_HandleTxComplete(UART_HandleTypeDef *huart);
uint8_t Motor_Transport_HandleError(UART_HandleTypeDef *huart);

#endif
