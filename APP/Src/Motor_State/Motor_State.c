#include "Motor_State.h"

#include <string.h>

#define MOTOR_STATE_PI      3.14159265358979323846f
#define MOTOR_STATE_TWO_PI  6.28318530717958647692f

static float normalized_direction(float direction)
{
  return direction < 0.0f ? -1.0f : 1.0f;
}

static float absolute_value(float value)
{
  return value < 0.0f ? -value : value;
}

/* Only the zero check treats a whole encoder turn as equivalent.  Position
 * feedback and joint position remain multi-turn values everywhere else. */
static float wrapped_zero_error(float rotor_position, float calibration_zero)
{
  float error = rotor_position - calibration_zero;
  while (error > MOTOR_STATE_PI) error -= MOTOR_STATE_TWO_PI;
  while (error < -MOTOR_STATE_PI) error += MOTOR_STATE_TWO_PI;
  return error;
}

static void refresh_derived(Motor_StateTypeDef *state,
                            float reduction_ratio,
                            float zero_threshold)
{
  if (state == NULL) return;

  float joint_delta = state->rotor_position - state->zero_rotor_position;
  float zero_error = wrapped_zero_error(state->rotor_position,
                                        state->zero_rotor_position);
  state->joint_angle = (reduction_ratio > 0.0f)
                           ? state->direction * joint_delta / reduction_ratio
                           : 0.0f;
  state->zero_checked = (state->online == Motor_Online &&
                         state->angle_valid == Motor_Angle_Valid &&
                         absolute_value(zero_error) <= zero_threshold) ? 1U : 0U;
}

void Motor_State_Init(Motor_StateTypeDef *state, float zero_offset, float direction)
{
  if (state == NULL) return;
  memset(state, 0, sizeof(*state));
  state->zero_rotor_position = zero_offset;
  state->direction = normalized_direction(direction);
  state->angle_valid = Motor_Angle_Invalid;
  state->online = Motor_Offline;
  state->handshake_status = Motor_Handshake_Timeout;
  state->target_active = Motor_Target_Inactive;
  state->target_result = Motor_Target_Idle;
}

void Motor_State_SetCalibration(Motor_StateTypeDef *state,
                                float zero_offset,
                                float direction,
                                float reduction_ratio,
                                float zero_threshold)
{
  if (state == NULL) return;
  state->zero_rotor_position = zero_offset;
  state->direction = normalized_direction(direction);
  refresh_derived(state, reduction_ratio, zero_threshold);
}

void Motor_State_UpdateRawFeedback(Motor_StateTypeDef *state,
                                   float rotor_position,
                                   float raw_velocity,
                                   float raw_torque,
                                   int8_t temperature_c,
                                   uint8_t motor_error,
                                   uint32_t timestamp,
                                   float reduction_ratio,
                                   float zero_threshold)
{
  if (state == NULL) return;

  state->rotor_position = rotor_position;
  state->raw_velocity = raw_velocity;
  state->raw_torque = raw_torque;
  state->temperature_c = temperature_c;
  state->motor_error = motor_error;
  state->timestamp = timestamp;
  state->angle_valid = Motor_Angle_Valid;
  state->online = Motor_Online;
  refresh_derived(state, reduction_ratio, zero_threshold);
}

void Motor_State_MarkOffline(Motor_StateTypeDef *state)
{
  if (state == NULL) return;
  state->online = Motor_Offline;
  state->angle_valid = Motor_Angle_Invalid;
  state->zero_checked = 0U;
}

void Motor_State_RecordError(Motor_StateTypeDef *state)
{
  if (state == NULL) return;
  if (state->error_count < UINT32_MAX) ++state->error_count;
}

float Motor_State_GetZeroError(const Motor_StateTypeDef *state)
{
  if (state == NULL) return 0.0f;
  return wrapped_zero_error(state->rotor_position, state->zero_rotor_position);
}

void Motor_State_GetSnapshot(const Motor_StateTypeDef *state,
                             Motor_StateSnapshotTypeDef *snapshot)
{
  if (state == NULL || snapshot == NULL) return;
  snapshot->rotor_position = state->rotor_position;
  snapshot->raw_velocity = state->raw_velocity;
  snapshot->raw_torque = state->raw_torque;
  snapshot->temperature_c = state->temperature_c;
  snapshot->motor_error = state->motor_error;
  snapshot->zero_rotor_position = state->zero_rotor_position;
  snapshot->direction = state->direction;
  snapshot->joint_position = state->joint_position;
  snapshot->online = (uint8_t)state->online;
  snapshot->angle_valid = (uint8_t)state->angle_valid;
  snapshot->zero_checked = state->zero_checked;
  snapshot->timestamp = state->timestamp;
  snapshot->error_count = state->error_count;
}
