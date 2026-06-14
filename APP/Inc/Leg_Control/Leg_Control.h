/**
******************************************************************************
  * @file    Leg_Control.h
  * @author  Si-Ghost
  * @brief   Head file of Leg_Control.c
  ******************************************************************************
  */

#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H

#include "GO-M8010-6.h"
#include "main.h"

typedef enum
{
  Leg_TX_M0,
  Leg_RX_M0,
  Leg_TX_M1,
  Leg_RX_M1,
  Leg_Done
} Leg_StatusTypeDef;

// 应该要有一个句柄用于操作腿部
typedef struct __Leg_HandlerTypeDef {
  MOTOR_send motor_cmd[2];
  MOTOR_recv motor_data[2];
  float P_init[2];
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  UART_HandleTypeDef* huartx;
  Leg_StatusTypeDef Leg_Status;
}Leg_HandlerTypeDef;

void Leg_Control_Start(void);
void Leg_TxCpltCallback(UART_HandleTypeDef *huart);
void Leg_RxCpltCallback(UART_HandleTypeDef *huart);

#endif //PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
