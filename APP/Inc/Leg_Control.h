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

// 应该要有一个句柄用于操作腿部
typedef struct __Leg_HandlerTypeDef {
  MOTOR_send motor0_send_data;
  MOTOR_send motor1_send_data;
  MOTOR_recv motor0_recv_data;
  MOTOR_recv motor1_recv_data;
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  UART_HandleTypeDef huartx;
}Leg_HandlerTypeDef;

#endif //PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
