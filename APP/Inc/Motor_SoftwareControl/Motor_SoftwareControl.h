#ifndef PARALLEL_QUADRUPED_ROBOT_MOTOR_SOFTWARE_CONTROL_H
#define PARALLEL_QUADRUPED_ROBOT_MOTOR_SOFTWARE_CONTROL_H

#include <stdint.h>

typedef enum
{
  Motor_SoftwareControl_Disabled = 0,
  Motor_SoftwareControl_Armed = 1,
  Motor_SoftwareControl_DryRun = 2,
  Motor_SoftwareControl_Stopped = 3,
  Motor_SoftwareControl_ActiveTorque = 4,
  Motor_SoftwareControl_CascadeDryRun = 5,
  Motor_SoftwareControl_CascadeActiveTorque = 6,
  Motor_SoftwareControl_StaticHoldDryRun = 7,
  Motor_SoftwareControl_StaticHoldActiveTorque = 8
} Motor_SoftwareControlMode;

typedef enum
{
  Motor_SoftwareControl_TestNone = 0,
  Motor_SoftwareControl_TestPositionStep = 1,
  Motor_SoftwareControl_TestStaticHold = 2
} Motor_SoftwareControlTest;

typedef enum
{
  Motor_SoftwareControl_LimitNone = 0,
  Motor_SoftwareControl_LimitPositionExcursion = 1,
  Motor_SoftwareControl_LimitRawVelocity = 2
} Motor_SoftwareControlSafetyLimit;

typedef enum
{
  Motor_SoftwareControl_StopNone = 0,
  Motor_SoftwareControl_StopOperator = 1,
  Motor_SoftwareControl_StopRescan = 2,
  Motor_SoftwareControl_StopOffline = 3,
  Motor_SoftwareControl_StopTransport = 4,
  Motor_SoftwareControl_StopInvalidCommand = 5,
  Motor_SoftwareControl_StopDuration = 6,
  Motor_SoftwareControl_StopSafetyLimit = 7,
  Motor_SoftwareControl_StopInvalidDt = 8,
  Motor_SoftwareControl_StopInvalidNumber = 9,
  Motor_SoftwareControl_StopCommandLink = 10
} Motor_SoftwareControlStopReason;

typedef struct
{
  Motor_SoftwareControlMode mode;
  Motor_SoftwareControlStopReason stop_reason;
  Motor_SoftwareControlTest test;
  Motor_SoftwareControlSafetyLimit safety_limit;
  int8_t motor_index;
  float arm_position;
  float raw_target;
  float ramped_target;
  float ramp_velocity;
  float actual_position;
  float position_error;
  float raw_velocity;
  float filtered_velocity;
  float feedback_torque;
  float velocity_filter_hz;
  float integral_position_gate;
  float kp;
  float ki;
  float kd;
  float p_term;
  float i_term;
  float d_term;
  float calculated_torque;
  float limited_torque;
  float torque_limit;
  float target_velocity_limit;
  float actual_velocity_limit;
  float position_excursion_limit;
  float speed_target;
  float speed_error;
  float position_loop_kp;
  float position_loop_ki;
  float position_loop_kd;
  float speed_loop_kp;
  float speed_loop_ki;
  float speed_loop_kd;
  float dt_s;
  uint32_t duration_ms;
  uint32_t elapsed_ms;
  uint32_t feedback_timestamp;
  uint8_t torque_limited;
  uint8_t dry_run;
  uint8_t integral_enabled;
} Motor_SoftwareControlSnapshot;

void Motor_SoftwareControl_Init(void);
int Motor_SoftwareControl_Arm(uint8_t motor_index, float rotor_position,
                              uint32_t feedback_timestamp);
int Motor_SoftwareControl_StartDryRun(uint8_t motor_index, float offset_rad,
                                      float kp, float kd, uint32_t duration_ms,
                                      uint32_t now_ms);
int Motor_SoftwareControl_StartStaticHoldDryRun(uint8_t motor_index,
                                                uint32_t now_ms);
int Motor_SoftwareControl_StartStaticHoldActive(uint8_t motor_index,
                                                uint32_t now_ms);
void Motor_SoftwareControl_Update(float rotor_position, float rotor_velocity,
                                  float feedback_torque,
                                  uint32_t feedback_timestamp, uint32_t now_ms);
void Motor_SoftwareControl_Stop(Motor_SoftwareControlStopReason reason);
void Motor_SoftwareControl_GetSnapshot(Motor_SoftwareControlSnapshot *snapshot);
int Motor_SoftwareControl_GetAuthorizedTorque(uint8_t motor_index, float *torque);
const char *Motor_SoftwareControl_GetLastRejectReason(void);

#endif
