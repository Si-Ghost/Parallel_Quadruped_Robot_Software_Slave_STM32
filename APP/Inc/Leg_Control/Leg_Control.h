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
  float x;
  float y;
} Leg_PointTypeDef;

typedef struct
{
  float theta1;                                      // 主动杆 AB 的机构角，单位 rad，对应 motor1。
  float theta2;                                      // 主动杆 AD 的机构角，单位 rad，对应 motor0。
} Leg_JointAnglesTypeDef;

typedef struct
{
  float angle;                                      // 最新反馈角，电机转子侧，单位 rad。
  Motor_AngleValidTypeDef angle_valid;              // 当前 angle 是否来自有效反馈。
  Motor_OnlineStateTypeDef online;                  // 该电机是否通过最近一次握手。
  Motor_HandshakeStatusTypeDef handshake_status;    // 最近一次握手结果。
  float target_offset;                              // Web 调试目标，相对握手角的转子侧偏移，单位 rad。
  Motor_TargetActiveTypeDef target_active;          // 当前是否正在执行 Web 调试目标。
  Motor_TargetResultTypeDef target_result;          // 最近一次 Web 调试目标结果。
  float target_start_angle;
  float target_kp;
  float target_kw;
  float target_hold_kp;
  float target_hold_kw;
  uint8_t target_hold_on_error;
  uint32_t target_start_tick;                       // 当前目标开始时的 HAL tick。
  uint32_t target_progress_tick;                    // 目标误差最近一次明显改善时的 HAL tick。
  float target_last_abs_error;                      // 用于堵转检测的上一次绝对误差。
  float target_stop_error;                          // 当前调试目标的完成误差阈值，转子侧 rad。
  uint32_t debug_last_log_tick;                     // 目标调试日志限频用 tick。
  uint32_t io_error_last_log_tick;                  // 电机 I/O 错误日志限频用 tick。
  uint8_t io_error_count;                           // 连续 I/O 失败计数，用于断联保护。
} Motor_RuntimeStateTypeDef;

typedef struct
{
  MOTOR_send motor_cmd[2];
  MOTOR_recv motor_data[2];
  Motor_RuntimeStateTypeDef motor_state[2];
  float p_init[2];
  float rotor_zero_offset[2];                       // 机构零位对应的电机转子侧角度，单位 rad。
  float motor_direction[2];                         // 转子角正方向到机构角正方向的符号，取 1 或 -1。
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
  UART_HandleTypeDef* huartx;
  Leg_StatusTypeDef Leg_Status;
  Motor_OnlineStateTypeDef has_online_motor;        // 这条腿至少有一个电机通过握手。
} Leg_HandlerTypeDef;

void Leg_Control_Start(void);
void Leg_Tx_Handler(Leg_HandlerTypeDef *hleg);
void Leg_Rx_Handler(Leg_HandlerTypeDef *hleg, uint16_t Size);
void Leg_Control_InitSafe(void);
void Leg_Control_Handshake(void);
void Leg_Control_RequestHandshake(void);
void Leg_Control_Service(uint32_t now_ms);
int  Leg_Control_SetDebugAngle(uint8_t motor_index, float angle_rad);
int  Leg_Control_SetDebugFootOffset(uint8_t leg, float dx_mm, float dy_mm);
int  Leg_Control_StartDebugTrace(uint8_t leg);
int  Leg_Control_StartAllMicroTest(void);
int  Leg_Control_StartPrepPoseTest(void);
int  Leg_Control_StartStandStepTest(void);
int  Leg_Control_StartTouchStepTest(void);
int  Leg_Control_StartLoadedStepTest(void);
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
int  Leg_Kinematics_Forward(const Leg_JointAnglesTypeDef *angles, Leg_PointTypeDef *foot);
int  Leg_Kinematics_Inverse(const Leg_PointTypeDef *foot, Leg_JointAnglesTypeDef *angles);

#endif //PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
