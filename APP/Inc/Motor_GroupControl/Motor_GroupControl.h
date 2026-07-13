#ifndef PARALLEL_QUADRUPED_ROBOT_MOTOR_GROUP_CONTROL_H
#define PARALLEL_QUADRUPED_ROBOT_MOTOR_GROUP_CONTROL_H

#include "Motor_State.h"

#include <stdint.h>

typedef enum
{
  Motor_Group_Disabled = 0,
  Motor_Group_Armed = 1,
  Motor_Group_Active = 2,
  Motor_Group_Stopped = 3
} Motor_GroupMode;

typedef enum
{
  Motor_Group_ProfileUniformOffset = 0,
  Motor_Group_ProfileStandPose = 1
} Motor_GroupProfile;

typedef enum
{
  Motor_Group_StopNone = 0,
  Motor_Group_StopOperator = 1,
  Motor_Group_StopInvalidState = 2,
  Motor_Group_StopOffline = 3,
  Motor_Group_StopCommandLink = 4,
  Motor_Group_StopTemperature = 5,
  Motor_Group_StopMotorFault = 6,
  Motor_Group_StopPositionLimit = 7,
  Motor_Group_StopController = 8,
  Motor_Group_StopStall = 9,
  Motor_Group_StopRescan = 10,
  Motor_Group_StopTransport = 11
} Motor_GroupStopReason;

typedef struct
{
  Motor_GroupMode mode;
  Motor_GroupProfile profile;
  Motor_GroupStopReason reason;
  uint8_t ready;
  uint8_t all_at_zero;
  float target_offset;
  float arm_position[8];
  float target_position[8];
  float actual_position[8];
  float position_error[8];
} Motor_GroupSnapshot;

typedef struct
{
  float position_peak_to_peak[8];
  float max_abs_velocity[8];
  float max_abs_torque[8];
} Motor_GroupDiagnostics;

void Motor_GroupControl_Init(void);
int Motor_GroupControl_ArmZero(const Motor_StateSnapshotTypeDef states[8],
                               uint32_t now_ms);
int Motor_GroupControl_ArmTarget(const Motor_StateSnapshotTypeDef states[8],
                                 float target_offset, uint32_t now_ms);
int Motor_GroupControl_ArmOffsets(const Motor_StateSnapshotTypeDef states[8],
                                  const float target_offsets[8],
                                  Motor_GroupProfile profile,
                                  uint32_t now_ms);
int Motor_GroupControl_Start(uint32_t now_ms);
void Motor_GroupControl_Update(uint8_t motor_index,
                               float rotor_position,
                               float rotor_velocity,
                               uint32_t feedback_timestamp);
void Motor_GroupControl_Stop(Motor_GroupStopReason reason);
uint8_t Motor_GroupControl_IsArmed(void);
uint8_t Motor_GroupControl_IsActive(void);
int Motor_GroupControl_GetAuthorizedTorque(uint8_t motor_index, float *torque);
void Motor_GroupControl_GetSnapshot(Motor_GroupSnapshot *snapshot);
void Motor_GroupControl_GetDiagnostics(Motor_GroupDiagnostics *diagnostics,
                                       uint8_t reset_window);

#endif
