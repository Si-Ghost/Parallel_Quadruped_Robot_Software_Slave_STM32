#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_LEG_CONTROL_H

#include "GO-M8010-6.h"
#include "Leg_Kinematics.h"
#include "Motor_State.h"
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

/* PID bring-up safety state.  Position mode is only a planned target while
 * MOTOR_TRANSPORT_ZERO_OUTPUT_ONLY remains enabled in Motor_Transport.c. */
typedef enum
{
  Motor_Control_ZeroOutput = 0,
  Motor_Control_Observe = 1,
  Motor_Control_ArmedSingleMotor = 2,
  Motor_Control_SingleMotorPosition = 3,
  Motor_Control_SingleLegPosition = 4
} Motor_ControlModeTypeDef;

typedef enum
{
  Motor_Control_Reason_None = 0,
  Motor_Control_Reason_OperatorStop = 1,
  Motor_Control_Reason_Rescan = 2,
  Motor_Control_Reason_Offline = 3,
  Motor_Control_Reason_TransportError = 4,
  Motor_Control_Reason_InvalidCommand = 5
} Motor_ControlReasonTypeDef;

typedef struct
{
  Motor_ControlModeTypeDef mode;
  Motor_ControlReasonTypeDef reason;
  int8_t armed_motor_index;
  float target_rotor_position;
  float actual_rotor_position;
  float target_joint_position;
  float actual_joint_position;
  float position_error;
  float kp;
  float kw;
  uint32_t duration_ms;
  uint8_t zero_output_guard;
} Motor_ControlSnapshotTypeDef;

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
int  Leg_Control_ArmSingleMotor(uint8_t motor_index);
int  Leg_Control_PlanSingleMotor(uint8_t motor_index, float offset_rad,
                                  float kp, float kw, uint32_t duration_ms);
const char *Leg_Control_GetLastPlanRejectReason(void);
void Leg_Control_ForceZeroOutput(Motor_ControlReasonTypeDef reason);
void Leg_Control_GetControlSnapshot(Motor_ControlSnapshotTypeDef *snapshot);
void Leg_Control_LogFootSnapshot(void);
int  Leg_Control_HoldCurrentPosition(void);
void Leg_Control_StopAllDebugTargets(uint8_t reason);
void Leg_Control_GetAngles(float angles[8], uint8_t valid[8]);
int  Leg_Control_GetMotorStateSnapshot(uint8_t motor_index,
                                       Motor_StateSnapshotTypeDef *state);
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
