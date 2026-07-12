#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_STATE_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_STATE_H

#include <stdint.h>

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

/*
 * Canonical motor feedback state.  The anonymous union keeps the previously
 * published Leg_Control field names source-compatible while making the
 * multi-turn rotor and joint domains explicit.
 *
 * GO-M8010 feedback pos is a signed int32 Q15 multi-turn rotor position.
 * Do not wrap it to [-pi, pi]: one rotor revolution is a real 2*pi / 6.33
 * joint displacement, not an equivalent mechanical pose.
 *
 * The target/debug fields remain here only as a compatibility tail during the
 * staged refactor.  Motor_State.c never reads or writes that command state.
 */
typedef struct
{
  union {
    float rotor_position;
    float raw_position;
    float angle;
  };
  union {
    float raw_velocity;
    float speed;
  };
  float raw_torque;
  union {
    float zero_rotor_position;
    float zero_offset;
  };
  float direction;
  union {
    float joint_position;
    float joint_angle;
  };
  Motor_AngleValidTypeDef angle_valid;
  Motor_OnlineStateTypeDef online;
  uint8_t zero_reference_valid;
  uint8_t zero_checked;
  uint32_t timestamp;
  uint32_t error_count;

  /* Leg_Control compatibility state; scheduled for separation in phase 2. */
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
} Motor_StateTypeDef;

/* Existing Leg_Control/Leg_Gait source API remains valid during migration. */
typedef Motor_StateTypeDef Motor_RuntimeStateTypeDef;

typedef struct
{
  float rotor_position;
  float raw_velocity;
  float raw_torque;
  float zero_rotor_position;
  float direction;
  float joint_position;
  uint8_t online;
  uint8_t angle_valid;
  uint8_t zero_checked;
  uint32_t timestamp;
  uint32_t error_count;
} Motor_StateSnapshotTypeDef;

void Motor_State_Init(Motor_StateTypeDef *state, float zero_offset, float direction);
void Motor_State_SetCalibration(Motor_StateTypeDef *state,
                                float zero_offset,
                                float direction,
                                float reduction_ratio,
                                float zero_threshold);
void Motor_State_UpdateRawFeedback(Motor_StateTypeDef *state,
                                   float rotor_position,
                                   float raw_velocity,
                                   float raw_torque,
                                   uint32_t timestamp,
                                   float reduction_ratio,
                                   float zero_threshold);
void Motor_State_MarkOffline(Motor_StateTypeDef *state);
void Motor_State_RecordError(Motor_StateTypeDef *state);
float Motor_State_GetZeroError(const Motor_StateTypeDef *state);
void Motor_State_GetSnapshot(const Motor_StateTypeDef *state,
                             Motor_StateSnapshotTypeDef *snapshot);

#endif
