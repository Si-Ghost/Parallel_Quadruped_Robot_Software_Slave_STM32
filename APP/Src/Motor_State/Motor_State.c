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

float Motor_State_WrapDelta(float angle, float reference)
{
  float delta = angle - reference;
  while (delta > MOTOR_STATE_PI) delta -= MOTOR_STATE_TWO_PI;
  while (delta < -MOTOR_STATE_PI) delta += MOTOR_STATE_TWO_PI;
  return delta;
}

static void refresh_derived(Motor_StateTypeDef *state,
                            float reduction_ratio,
                            float zero_threshold)
{
  if (state == NULL) return;

  float zero_error = Motor_State_WrapDelta(state->single_turn_angle,
                                           state->zero_offset);
  state->joint_angle = (reduction_ratio > 0.0f)
                           ? state->direction * zero_error / reduction_ratio
                           : 0.0f;
  state->zero_checked = (state->online == Motor_Online &&
                         state->angle_valid == Motor_Angle_Valid &&
                         absolute_value(zero_error) <= zero_threshold) ? 1U : 0U;
}

void Motor_State_Init(Motor_StateTypeDef *state, float zero_offset, float direction)
{
  if (state == NULL) return;
  memset(state, 0, sizeof(*state));
  state->zero_offset = zero_offset;
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
  state->zero_offset = zero_offset;
  state->direction = normalized_direction(direction);
  refresh_derived(state, reduction_ratio, zero_threshold);
}

void Motor_State_UpdateRawFeedback(Motor_StateTypeDef *state,
                                   float raw_position,
                                   float raw_velocity,
                                   float raw_torque,
                                   uint32_t timestamp,
                                   float reduction_ratio,
                                   float zero_threshold)
{
  if (state == NULL) return;

  if (state->angle_valid == Motor_Angle_Valid) {
    state->accumulated_angle += Motor_State_WrapDelta(raw_position,
                                                      state->single_turn_angle);
  } else {
    state->accumulated_angle = state->zero_offset +
                               Motor_State_WrapDelta(raw_position,
                                                     state->zero_offset);
  }

  state->raw_position = raw_position;
  state->raw_velocity = raw_velocity;
  state->raw_torque = raw_torque;
  state->single_turn_angle = raw_position;
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
  return Motor_State_WrapDelta(state->single_turn_angle, state->zero_offset);
}

void Motor_State_GetSnapshot(const Motor_StateTypeDef *state,
                             Motor_StateSnapshotTypeDef *snapshot)
{
  if (state == NULL || snapshot == NULL) return;
  snapshot->raw_position = state->raw_position;
  snapshot->raw_velocity = state->raw_velocity;
  snapshot->raw_torque = state->raw_torque;
  snapshot->single_turn_angle = state->single_turn_angle;
  snapshot->accumulated_angle = state->accumulated_angle;
  snapshot->zero_offset = state->zero_offset;
  snapshot->direction = state->direction;
  snapshot->joint_angle = state->joint_angle;
  snapshot->online = (uint8_t)state->online;
  snapshot->angle_valid = (uint8_t)state->angle_valid;
  snapshot->zero_checked = state->zero_checked;
  snapshot->timestamp = state->timestamp;
  snapshot->error_count = state->error_count;
}
