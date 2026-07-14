#ifndef PARALLEL_QUADRUPED_ROBOT_MOTOR_LEG_TRAJECTORY_H
#define PARALLEL_QUADRUPED_ROBOT_MOTOR_LEG_TRAJECTORY_H

#include "Leg_Kinematics.h"
#include "Motor_State.h"

#include <stdint.h>

#define MOTOR_LEG_TRAJECTORY_LEG_INDEX          1U
#define MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX  2U
#define MOTOR_LEG_TRAJECTORY_ACTIVE_ENABLED      1U
#define MOTOR_LEG_TRAJECTORY_LEVEL_MIN            1U
#define MOTOR_LEG_TRAJECTORY_LEVEL_MAX            2U
#define MOTOR_LEG_TRAJECTORY_TORQUE_MAX_NM        1.0f
#define MOTOR_LEG_TRAJECTORY_TARGET_SPEED_MAX     2.0f
#define MOTOR_LEG_TRAJECTORY_ACTUAL_SPEED_MAX     3.0f
#define MOTOR_LEG_TRAJECTORY_TRACKING_ERROR_MAX    0.35f
#define MOTOR_LEG_TRAJECTORY_POSITION_KP          35.9f
#define MOTOR_LEG_TRAJECTORY_POSITION_KD           1.0f
#define MOTOR_LEG_TRAJECTORY_SPEED_KP              0.01f
#define MOTOR_LEG_TRAJECTORY_SPEED_KI              0.0006f
#define MOTOR_LEG_TRAJECTORY_SPEED_KD              0.0015f
#define MOTOR_LEG_TRAJECTORY_INTEGRAL_MAX           0.20f

typedef struct
{
  uint8_t level;
  float lift_mm;
  uint32_t period_ms;
  uint32_t settle_ms;
  uint32_t duration_ms;
  float position_max;
  float target_delta_max;
} Motor_LegTrajectoryProfile;

typedef enum
{
  Motor_LegTrajectory_Disabled = 0,
  Motor_LegTrajectory_Armed = 1,
  Motor_LegTrajectory_DryRun = 2,
  Motor_LegTrajectory_Active = 3,
  Motor_LegTrajectory_Stopped = 4,
  Motor_LegTrajectory_Hold = 5
} Motor_LegTrajectoryMode;

typedef enum
{
  Motor_LegTrajectory_StopNone = 0,
  Motor_LegTrajectory_StopOperator = 1,
  Motor_LegTrajectory_StopInvalidCommand = 2,
  Motor_LegTrajectory_StopOffline = 3,
  Motor_LegTrajectory_StopCommandLink = 4,
  Motor_LegTrajectory_StopTemperature = 5,
  Motor_LegTrajectory_StopMotorFault = 6,
  Motor_LegTrajectory_StopIk = 7,
  Motor_LegTrajectory_StopPosition = 8,
  Motor_LegTrajectory_StopVelocity = 9,
  Motor_LegTrajectory_StopController = 10,
  Motor_LegTrajectory_StopComplete = 11,
  Motor_LegTrajectory_StopTransport = 12,
  Motor_LegTrajectory_StopRescan = 13
} Motor_LegTrajectoryStopReason;

typedef struct
{
  Motor_LegTrajectoryMode mode;
  Motor_LegTrajectoryStopReason reason;
  uint8_t dry_run;
  uint8_t dry_run_passed;
  uint8_t trajectory_complete;
  uint8_t hold_current_position;
  Motor_LegTrajectoryProfile profile;
  int8_t stop_motor_index;
  float stop_detail;
  uint32_t stop_sequence;
  uint32_t elapsed_ms;
  float phase;
  Leg_PointTypeDef base_foot;
  Leg_PointTypeDef target_foot;
  float peak_target_delta[2];
  float arm_position[2];
  float zero_position[2];
  float direction[2];
  float arm_joint_position[2];
  float base_joint_position[2];
  int8_t arm_temperature_c[2];
  uint8_t arm_zero_checked[2];
  uint32_t arm_feedback_age_ms[2];
  float target_position[2];
  float actual_position[2];
  float position_error[2];
  float raw_velocity[2];
  float target_velocity[2];
  float speed_target[2];
  float speed_error[2];
  float p_term[2];
  float i_term[2];
  float d_term[2];
  float torque[2];
  uint8_t torque_limited[2];
  uint8_t overspeed_count[2];
  uint8_t tracking_error_count[2];
  uint32_t feedback_count[2];
  uint32_t torque_limit_count[2];
  float peak_abs_target_velocity[2];
  float peak_abs_actual_velocity[2];
  float peak_abs_position_error[2];
  float peak_abs_torque[2];
  float min_dt_ms[2];
  float max_dt_ms[2];
} Motor_LegTrajectorySnapshot;

void Motor_LegTrajectory_Init(void);
int Motor_LegTrajectory_Arm(const Motor_StateSnapshotTypeDef states[2],
                            uint8_t level,
                            uint32_t now_ms);
int Motor_LegTrajectory_Start(uint8_t dry_run, uint32_t now_ms);
int Motor_LegTrajectory_EnterHold(Motor_LegTrajectoryStopReason reason,
                                  uint8_t hold_current_position,
                                  float detail);
void Motor_LegTrajectory_Update(uint8_t motor_index,
                                float rotor_position,
                                float rotor_velocity,
                                uint32_t feedback_timestamp,
                                uint32_t now_ms);
void Motor_LegTrajectory_Stop(Motor_LegTrajectoryStopReason reason,
                              int8_t motor_index,
                              float detail);
uint8_t Motor_LegTrajectory_IsRunning(void);
int Motor_LegTrajectory_GetAuthorizedTorque(uint8_t motor_index,
                                            float *torque);
void Motor_LegTrajectory_GetSnapshot(Motor_LegTrajectorySnapshot *snapshot);

#endif
