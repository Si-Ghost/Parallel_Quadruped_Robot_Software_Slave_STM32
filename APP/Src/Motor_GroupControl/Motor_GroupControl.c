#include "Motor_GroupControl.h"

#include <math.h>
#include <string.h>

#define GROUP_MOTOR_COUNT                 8U
#define GROUP_TWO_PI                      6.28318530717958647692f
#define GROUP_MAX_TARGET_OFFSET_RAD       2.00f
#define GROUP_MAX_TARGET_DELTA_RAD        2.10f
#define GROUP_OFFSET_START_ZERO_RAD       0.10f
#define GROUP_MAX_ARM_EXCURSION_RAD       2.25f
#define GROUP_TARGET_SPEED_RAD_S          0.25f
#define GROUP_TARGET_ERROR_RAD            0.08f
#define GROUP_MIN_DT_S                    0.0002f
#define GROUP_MAX_DT_S                    0.0100f
#define GROUP_POSITION_KP                 35.9f
#define GROUP_POSITION_KD                 1.0f
/* Keep the cautious group target ramp above, but use the already validated
 * single-motor static-hold cascade limits once position error is generated. */
#define GROUP_SPEED_TARGET_MAX_RAD_S      100.0f
#define GROUP_SPEED_KP                    0.01f
#define GROUP_SPEED_KI                    0.0006f
#define GROUP_SPEED_KD                    0.0015f
#define GROUP_SPEED_INTEGRAL_MAX          0.20f
#define GROUP_TORQUE_MAX_NM               1.50f

typedef struct
{
  float arm_position;
  float target_position;
  float ramped_target;
  float actual_position;
  float position_error;
  float previous_position_error;
  float previous_speed_error;
  float integral;
  float torque;
  uint32_t previous_feedback_timestamp;
} Motor_GroupChannel;

static Motor_GroupChannel channels[GROUP_MOTOR_COUNT];
static Motor_GroupMode group_mode;
static Motor_GroupStopReason stop_reason;
static uint8_t group_ready;
static float group_target_offset;

static float clamp_symmetric(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static float aligned_zero(float arm_position, float calibration_zero)
{
  float turns = roundf((arm_position - calibration_zero) / GROUP_TWO_PI);
  return calibration_zero + turns * GROUP_TWO_PI;
}

void Motor_GroupControl_Init(void)
{
  memset(channels, 0, sizeof(channels));
  group_mode = Motor_Group_Disabled;
  stop_reason = Motor_Group_StopNone;
  group_ready = 0U;
  group_target_offset = 0.0f;
}

void Motor_GroupControl_Stop(Motor_GroupStopReason reason)
{
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    channels[i].torque = 0.0f;
    channels[i].integral = 0.0f;
  }
  group_mode = reason == Motor_Group_StopNone ? Motor_Group_Disabled
                                               : Motor_Group_Stopped;
  stop_reason = reason;
  group_ready = 0U;
}

int Motor_GroupControl_ArmZero(const Motor_StateSnapshotTypeDef states[8],
                               uint32_t now_ms)
{
  return Motor_GroupControl_ArmTarget(states, 0.0f, now_ms);
}

int Motor_GroupControl_ArmTarget(const Motor_StateSnapshotTypeDef states[8],
                                 float target_offset, uint32_t now_ms)
{
  if (states == NULL || !isfinite(target_offset) ||
      fabsf(target_offset) > GROUP_MAX_TARGET_OFFSET_RAD) return 0;

  Motor_GroupControl_Init();
  group_target_offset = target_offset;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    const Motor_StateSnapshotTypeDef *state = &states[i];
    if (!state->online || !state->angle_valid || state->timestamp == 0U ||
        (now_ms - state->timestamp) > 10U || !isfinite(state->rotor_position) ||
        state->motor_error != 0U || state->temperature_c >= 40) {
      Motor_GroupControl_Stop(state->motor_error != 0U
                                  ? Motor_Group_StopMotorFault
                                  : state->temperature_c >= 40
                                        ? Motor_Group_StopTemperature
                                        : Motor_Group_StopInvalidState);
      return 0;
    }

    Motor_GroupChannel *channel = &channels[i];
    channel->arm_position = state->rotor_position;
    float zero_position = aligned_zero(state->rotor_position,
                                       state->zero_rotor_position);
    /* Non-zero synchronized tests must begin from the verified zero pose.
     * Returning to zero remains allowed from either +1 or +2 rad. */
    if (fabsf(target_offset) > 0.0001f &&
        fabsf(zero_position - channel->arm_position) >
            GROUP_OFFSET_START_ZERO_RAD) {
      Motor_GroupControl_Stop(Motor_Group_StopInvalidState);
      return 0;
    }
    channel->target_position = zero_position + target_offset;
    float delta = channel->target_position - channel->arm_position;
    if (!isfinite(channel->target_position) ||
        fabsf(delta) > GROUP_MAX_TARGET_DELTA_RAD) {
      Motor_GroupControl_Stop(Motor_Group_StopPositionLimit);
      return 0;
    }
    channel->ramped_target = channel->arm_position;
    channel->actual_position = channel->arm_position;
    channel->position_error = delta;
  }

  group_mode = Motor_Group_Armed;
  stop_reason = Motor_Group_StopNone;
  group_ready = 1U;
  return 1;
}

int Motor_GroupControl_Start(uint32_t now_ms)
{
  (void)now_ms;
  if (group_mode != Motor_Group_Armed || !group_ready) return 0;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    channels[i].previous_feedback_timestamp = 0U;
    channels[i].previous_position_error = 0.0f;
    channels[i].previous_speed_error = 0.0f;
    channels[i].integral = 0.0f;
    channels[i].torque = 0.0f;
  }
  group_mode = Motor_Group_Active;
  stop_reason = Motor_Group_StopNone;
  return 1;
}

void Motor_GroupControl_Update(uint8_t motor_index,
                               float rotor_position,
                               float rotor_velocity,
                               uint32_t feedback_timestamp)
{
  if (group_mode != Motor_Group_Active || motor_index >= GROUP_MOTOR_COUNT)
    return;
  if (!isfinite(rotor_position) || !isfinite(rotor_velocity) ||
      feedback_timestamp == 0U) {
    Motor_GroupControl_Stop(Motor_Group_StopController);
    return;
  }

  Motor_GroupChannel *channel = &channels[motor_index];
  channel->actual_position = rotor_position;
  if (fabsf(rotor_position - channel->arm_position) >
      GROUP_MAX_ARM_EXCURSION_RAD) {
    Motor_GroupControl_Stop(Motor_Group_StopPositionLimit);
    return;
  }

  if (channel->previous_feedback_timestamp == 0U) {
    channel->previous_feedback_timestamp = feedback_timestamp;
    return;
  }
  if (feedback_timestamp == channel->previous_feedback_timestamp) return;

  float dt = (float)(feedback_timestamp - channel->previous_feedback_timestamp) * 0.001f;
  channel->previous_feedback_timestamp = feedback_timestamp;
  if (!isfinite(dt) || dt < GROUP_MIN_DT_S || dt > GROUP_MAX_DT_S) {
    Motor_GroupControl_Stop(Motor_Group_StopController);
    return;
  }

  float remaining = channel->target_position - channel->ramped_target;
  float target_step = clamp_symmetric(remaining, GROUP_TARGET_SPEED_RAD_S * dt);
  channel->ramped_target += target_step;
  float ramp_velocity = target_step / dt;

  float previous_position_error = channel->previous_position_error;
  channel->position_error = channel->ramped_target - rotor_position;
  channel->previous_position_error = channel->position_error;
  float speed_target = clamp_symmetric(
      ramp_velocity + GROUP_POSITION_KP * channel->position_error +
          GROUP_POSITION_KD * (channel->position_error - previous_position_error),
      GROUP_SPEED_TARGET_MAX_RAD_S);
  float speed_error = speed_target - rotor_velocity;
  float speed_delta = speed_error - channel->previous_speed_error;
  channel->previous_speed_error = speed_error;
  channel->integral = clamp_symmetric(
      channel->integral + GROUP_SPEED_KI * speed_error,
      GROUP_SPEED_INTEGRAL_MAX);
  channel->torque = clamp_symmetric(
      GROUP_SPEED_KP * speed_error + channel->integral +
          GROUP_SPEED_KD * speed_delta,
      GROUP_TORQUE_MAX_NM);

  /* Infinite hold intentionally does not treat persistent position error as a
   * stall.  An operator load may prevent progress while the controller must
   * continue producing restoring torque.  Temperature, hardware-fault,
   * feedback/link and arm-excursion interlocks remain responsible for stops. */
}

uint8_t Motor_GroupControl_IsArmed(void)
{
  return group_mode == Motor_Group_Armed ? 1U : 0U;
}

uint8_t Motor_GroupControl_IsActive(void)
{
  return group_mode == Motor_Group_Active ? 1U : 0U;
}

int Motor_GroupControl_GetAuthorizedTorque(uint8_t motor_index, float *torque)
{
  if (torque == NULL) return 0;
  *torque = 0.0f;
  if (group_mode != Motor_Group_Active || motor_index >= GROUP_MOTOR_COUNT)
    return 0;
  if (!isfinite(channels[motor_index].torque) ||
      fabsf(channels[motor_index].torque) > GROUP_TORQUE_MAX_NM) {
    Motor_GroupControl_Stop(Motor_Group_StopController);
    return 0;
  }
  *torque = channels[motor_index].torque;
  return 1;
}

void Motor_GroupControl_GetSnapshot(Motor_GroupSnapshot *snapshot)
{
  if (snapshot == NULL) return;
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->mode = group_mode;
  snapshot->reason = stop_reason;
  snapshot->ready = group_ready;
  snapshot->all_at_zero = group_mode == Motor_Group_Active ? 1U : 0U;
  snapshot->target_offset = group_target_offset;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    snapshot->arm_position[i] = channels[i].arm_position;
    snapshot->target_position[i] = channels[i].target_position;
    snapshot->actual_position[i] = channels[i].actual_position;
    snapshot->position_error[i] = channels[i].target_position -
                                  channels[i].actual_position;
    if (fabsf(snapshot->position_error[i]) > GROUP_TARGET_ERROR_RAD)
      snapshot->all_at_zero = 0U;
  }
}
