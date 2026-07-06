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

typedef enum
{
  Motor_Angle_Invalid = 0,
  Motor_Angle_Valid = 1
} Motor_AngleValidTypeDef;

typedef enum
{
  Motor_Offline = 0,
  Motor_Online = 1
} Motor_OnlineStateTypeDef;

typedef enum
{
  Motor_Handshake_Ok = 0,
  Motor_Handshake_Timeout = 1,
  Motor_Handshake_UartError = 2,
  Motor_Handshake_BadId = 3
} Motor_HandshakeStatusTypeDef;

typedef enum
{
  Motor_Target_Inactive = 0,
  Motor_Target_Active = 1
} Motor_TargetActiveTypeDef;

typedef enum
{
  Motor_Target_Idle = 0,
  Motor_Target_Running = 1,
  Motor_Target_Done = 2,
  Motor_Target_Timeout = 3,
  Motor_Target_Stall = 4,
  Motor_Target_Stopped = 5
} Motor_TargetResultTypeDef;

typedef struct
{
  float angle;                                      // Latest feedback angle, rotor side, rad.
  Motor_AngleValidTypeDef angle_valid;              // Whether angle contains a fresh valid feedback sample.
  Motor_OnlineStateTypeDef online;                  // Whether this motor passed the latest handshake.
  Motor_HandshakeStatusTypeDef handshake_status;    // Result of the latest handshake attempt.
  float target_offset;                              // Web target offset from handshake angle, rotor side, rad.
  Motor_TargetActiveTypeDef target_active;          // Whether a debug target is currently being driven.
  Motor_TargetResultTypeDef target_result;          // Last debug target outcome reported to ESP32/Web.
  uint32_t target_start_tick;                       // HAL tick when the current target started.
  uint32_t target_progress_tick;                    // HAL tick when target error last improved enough.
  float target_last_abs_error;                      // Error used for stall progress detection.
  uint32_t debug_last_log_tick;                     // Log throttle tick for target command debug output.
  uint32_t io_error_last_log_tick;                  // Log throttle tick for motor I/O errors.
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
  Motor_OnlineStateTypeDef online;
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
