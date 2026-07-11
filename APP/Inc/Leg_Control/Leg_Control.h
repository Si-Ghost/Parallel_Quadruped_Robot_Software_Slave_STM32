#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H

#include "GO-M8010-6.h"
#include "Leg_Kinematics.h"
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
  /* angle is the raw single-turn feedback used by control and zero checks. */
  float angle;
  /* display_angle is unwrapped at the 1 kHz transport rate for telemetry. */
  float display_angle;
  float speed;
  Motor_AngleValidTypeDef angle_valid;
  Motor_OnlineStateTypeDef online;
  Motor_HandshakeStatusTypeDef handshake_status;
  float target_offset;
  Motor_TargetActiveTypeDef target_active;
  Motor_TargetResultTypeDef target_result;
  float target_start_angle;
  float target_kp;
  float target_kw;
  float target_hold_kp;
  float target_hold_kw;
  uint8_t target_hold_on_error;
  uint32_t target_start_tick;
  uint32_t target_progress_tick;
  float target_last_abs_error;
  float target_stop_error;
  uint32_t debug_last_log_tick;
  uint32_t io_error_last_log_tick;
  uint8_t io_error_count;
} Motor_RuntimeStateTypeDef;

typedef struct
{
  MOTOR_send motor_cmd[2];
  MOTOR_recv motor_data[2];
  Motor_RuntimeStateTypeDef motor_state[2];
  float p_init[2];
  float rotor_zero_offset[2];
  float motor_direction[2];
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  UART_HandleTypeDef* huartx;
  Leg_StatusTypeDef Leg_Status;
  Motor_OnlineStateTypeDef has_online_motor;
} Leg_HandlerTypeDef;

#define LEG_WEB_KP                  2.0f
#define LEG_WEB_KW                  0.15f
#define LEG_HOLD_KP                 1.0f
#define LEG_HOLD_KW                 0.15f

/* --- core motor control (Leg_Control.c) --- */
void Leg_Control_Start(void);
void Leg_Tx_Handler(Leg_HandlerTypeDef *hleg);
void Leg_Rx_Handler(Leg_HandlerTypeDef *hleg, uint16_t Size);
void Leg_Control_InitSafe(void);
void Leg_Control_Handshake(void);
void Leg_Control_RequestHandshake(void);
void Leg_Control_Service(uint32_t now_ms);
int  Leg_Control_SetDebugAngle(uint8_t motor_index, float angle_rad);
void Leg_Control_LogFootSnapshot(void);
int  Leg_Control_HoldCurrentPosition(void);
void Leg_Control_StopAllDebugTargets(uint8_t reason);
void Leg_Control_GetAngles(float angles[8], uint8_t valid[8]);
void Leg_Control_GetOnline(uint8_t motor_online[8], uint8_t leg_online[4]);
void Leg_Control_GetHandshakeErrors(uint8_t motor_error[8]);
void Leg_Control_GetTargetStates(uint8_t active[8], uint8_t result[8]);
float Leg_Control_GetZeroThreshold(void);
void Leg_Control_GetZeroCheck(float zero_error[8], uint8_t zero_ok[8], uint8_t *all_zero_ok);
int  Leg_Control_SetZeroOffsets(uint8_t leg, const float rotor_zero_offset[2], const float motor_direction[2]);
int  Leg_Control_SetCurrentPositionAsZero(uint8_t leg);
int  Leg_Control_GetJointAngles(uint8_t leg, Leg_JointAnglesTypeDef *angles, uint8_t theta_valid[2]);
int  Leg_Control_JointToRotorTargets(uint8_t leg, const Leg_JointAnglesTypeDef *angles, float rotor_targets[2]);

/* --- shared helpers exported for Leg_Gait --- */
uint8_t Leg_Control_MotorIndex(uint8_t leg, uint8_t motor);
Motor_RuntimeStateTypeDef *Leg_Control_MotorState(uint8_t motor_index);
float Leg_Control_NormalizedMotorDir(float direction);
int   Leg_Control_GetCurrentFoot(uint8_t leg, Leg_PointTypeDef *foot);
int   Leg_Control_ComputeFootTargetOffsets(uint8_t leg, const Leg_PointTypeDef *target_foot, float offsets[2]);
int   Leg_Control_ApplyDebugTarget(uint8_t motor_index, uint8_t force_log);
void  Leg_Control_StopDebugTarget(uint8_t motor_index, Motor_TargetResultTypeDef result);
void  Leg_Control_StartOffset(uint8_t motor_index, float offset);
void  Leg_Control_StartOffsetWithStopError(uint8_t motor_index, float offset, float stop_error);
int   Leg_Control_SetDebugFootOffset(uint8_t leg, float dx_mm, float dy_mm);
int   Leg_Control_AppendFixed4(char *buf, size_t size, int pos, float value);

#endif
