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
  Leg_Idle,
  Leg_TX_M0,
  Leg_RX_M0,
  Leg_TX_M1,
  Leg_RX_M1,
  Leg_Done
} Leg_StatusTypeDef;

typedef struct
{
  float angle;
  uint8_t angle_valid;
  uint8_t online;
  uint8_t handshake_error;
  float target_offset;
  uint8_t target_active;
  uint8_t target_result;
  uint32_t target_start_tick;
  uint32_t target_progress_tick;
  float target_last_abs_error;
  uint32_t debug_last_log_tick;
  uint32_t io_error_last_log_tick;
} Motor_RuntimeStateTypeDef;

typedef struct
{
  MOTOR_send motor_cmd[2];
  MOTOR_recv motor_data[2];
  Motor_RuntimeStateTypeDef motor_state[2];
  float p_init[2];
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  UART_HandleTypeDef* huartx;
  Leg_StatusTypeDef Leg_Status;
  uint8_t online;
} Leg_HandlerTypeDef;

void Leg_Control_Start(void);
void Leg_Tx_Handler(Leg_HandlerTypeDef *hleg);
void Leg_Rx_Handler(Leg_HandlerTypeDef *hleg, uint16_t Size);
void Leg_Control_InitSafe(void);
void Leg_Control_Handshake(void);
void Leg_Control_RequestHandshake(void);
void Leg_Control_Service(uint32_t now_ms);
int  Leg_Control_SetDebugAngle(uint8_t motor_index, float angle_rad);
void Leg_Control_StopAllDebugTargets(uint8_t reason);
void Leg_Control_GetAngles(float angles[8], uint8_t valid[8]);
void Leg_Control_GetOnline(uint8_t motor_online[8], uint8_t leg_online[4]);
void Leg_Control_GetHandshakeErrors(uint8_t motor_error[8]);
void Leg_Control_GetTargetStates(uint8_t active[8], uint8_t result[8]);

#endif //PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
