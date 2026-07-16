#include "Motor_GroupControl.h"

#include <math.h>
#include <string.h>

#define GROUP_MOTOR_COUNT                 8U
#define GROUP_TWO_PI                      6.28318530717958647692f
#define GROUP_MAX_TARGET_OFFSET_RAD       2.00f
#define GROUP_MAX_TARGET_DELTA_RAD        2.60f
#define GROUP_OFFSET_START_ZERO_RAD       0.50f
#define GROUP_MAX_ZERO_EXCURSION_RAD      2.25f
#define GROUP_ZERO_RETURN_EXCURSION_RAD   3.50f
#define GROUP_GAIT_DEFAULT_MAX_ZERO_EXCURSION_RAD 4.00f
#define GROUP_TARGET_SPEED_RAD_S          0.25f
#define GROUP_STAND_TARGET_SPEED_RAD_S    0.125f
#define GROUP_TARGET_ERROR_RAD            0.08f
#define GROUP_GAIT_CONFIG_MIN_EXCURSION_RAD 3.50f
#define GROUP_GAIT_CONFIG_MAX_EXCURSION_RAD 4.50f
#define GROUP_MIN_DT_S                    0.0002f
#define GROUP_MAX_DT_S                    \
  ((float)MOTOR_GROUP_ACTIVE_FEEDBACK_TIMEOUT_MS * 0.001f)
#define GROUP_POSITION_KP                 35.9f
#define GROUP_POSITION_KD                 1.0f
#define GROUP_SPEED_TARGET_MAX_RAD_S      100.0f
#define GROUP_GAIT_ANGLE_OUT_MAX_RAD_S  10000.0f
#define GROUP_SPEED_KP                    0.06f
#define GROUP_SPEED_KI                    0.0036f
#define GROUP_SPEED_KD                    0.0090f
#define GROUP_SPEED_INTEGRAL_MAX          0.60f
#define GROUP_HOLD_POSITION_KP_NM_RAD     1.00f
#define GROUP_HOLD_ENTRY_ERROR_RAD        0.0001f
#define GROUP_TORQUE_MAX_NM               1.50f
/* Software_Ref SPEED_MAX_OUT is the final torque command limit. */
#define GROUP_GAIT_TORQUE_MAX_NM          3.50f

typedef struct
{
  float arm_position;
  float zero_position;
  float target_position;
  float pending_arm_position;
  float pending_target_position;
  float ramped_target;
  float actual_position;
  float actual_velocity;
  float position_error;
  float previous_position_error;
  float previous_speed_error;
  float integral;
  float torque;
  float diagnostic_position_min;
  float diagnostic_position_max;
  float diagnostic_max_abs_velocity;
  float diagnostic_max_abs_torque;
  uint32_t previous_feedback_timestamp;
} Motor_GroupChannel;

static Motor_GroupChannel channels[GROUP_MOTOR_COUNT];
static Motor_GroupMode group_mode;
static Motor_GroupProfile group_profile;
static Motor_GroupStopReason stop_reason;
static uint8_t group_ready;
static float group_target_offset;
static Motor_GroupProfile pending_group_profile;
static float pending_group_target_offset;
static int8_t stop_motor_index;
static float stop_detail;
static uint32_t stop_sequence;
static Motor_GroupGaitLimits gait_limits;

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
  group_profile = Motor_Group_ProfileUniformOffset;
  stop_reason = Motor_Group_StopNone;
  group_ready = 0U;
  group_target_offset = 0.0f;
  pending_group_profile = Motor_Group_ProfileUniformOffset;
  pending_group_target_offset = 0.0f;
  stop_motor_index = -1;
  stop_detail = 0.0f;
  gait_limits.max_zero_excursion_rad =
      GROUP_GAIT_DEFAULT_MAX_ZERO_EXCURSION_RAD;
}

void Motor_GroupControl_Stop(Motor_GroupStopReason reason)
{
  Motor_GroupControl_StopWithContext(reason, -1, 0.0f);
}

void Motor_GroupControl_StopWithContext(Motor_GroupStopReason reason,
                                        int8_t motor_index,
                                        float detail)
{
  /* Preserve the first non-zero stop.  Leg_Control_ForceZeroOutput() also
   * calls the generic stop path while clearing every command; without this
   * guard it overwrote the actual temperature/offline/controller cause. */
  if (reason != Motor_Group_StopNone && group_mode == Motor_Group_Stopped)
    return;

  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    channels[i].torque = 0.0f;
    channels[i].integral = 0.0f;
  }
  if (reason == Motor_Group_StopNone) {
    group_mode = Motor_Group_Disabled;
    stop_motor_index = -1;
    stop_detail = 0.0f;
  } else {
    stop_motor_index = motor_index;
    stop_detail = isfinite(detail) ? detail : 0.0f;
    ++stop_sequence;
    group_mode = Motor_Group_Stopped;
  }
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
  if (!isfinite(target_offset)) return 0;
  float target_offsets[GROUP_MOTOR_COUNT];
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i)
    target_offsets[i] = target_offset;
  return Motor_GroupControl_ArmOffsets(states, target_offsets,
                                       Motor_Group_ProfileUniformOffset,
                                       now_ms);
}

int Motor_GroupControl_ArmOffsets(const Motor_StateSnapshotTypeDef states[8],
                                  const float target_offsets[8],
                                  Motor_GroupProfile profile,
                                  uint32_t now_ms)
{
  if (states == NULL || target_offsets == NULL ||
      (profile != Motor_Group_ProfileUniformOffset &&
       profile != Motor_Group_ProfileStandPose)) return 0;

  uint8_t active_retarget =
      (group_mode == Motor_Group_Active ||
       group_mode == Motor_Group_ActivePending) ? 1U : 0U;
  uint8_t requests_nonzero = profile == Motor_Group_ProfileStandPose ? 1U : 0U;
  float arm_positions[GROUP_MOTOR_COUNT];
  float zero_positions[GROUP_MOTOR_COUNT];
  float new_targets[GROUP_MOTOR_COUNT];

  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    if (fabsf(target_offsets[i]) > 0.0001f) requests_nonzero = 1U;
  }
  /* A non-zero retarget may only be staged while the currently executing
   * controller is still holding mechanical zero.  Returning to zero from a
   * non-zero target remains allowed. */
  if (active_retarget != 0U && requests_nonzero != 0U &&
      (group_profile != Motor_Group_ProfileUniformOffset ||
       fabsf(group_target_offset) > 0.0001f))
    return 0;

  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    float target_offset = target_offsets[i];
    if (!isfinite(target_offset) ||
        fabsf(target_offset) > GROUP_MAX_TARGET_OFFSET_RAD) {
      if (active_retarget == 0U)
        Motor_GroupControl_StopWithContext(Motor_Group_StopPositionLimit,
                                           (int8_t)i, target_offset);
      return 0;
    }
    const Motor_StateSnapshotTypeDef *state = &states[i];
    if (!state->online || !state->angle_valid || state->timestamp == 0U ||
        (now_ms - state->timestamp) > 10U || !isfinite(state->rotor_position) ||
        state->motor_error != 0U || state->temperature_c >= 40) {
      Motor_GroupStopReason reason = state->motor_error != 0U
                                         ? Motor_Group_StopMotorFault
                                         : state->temperature_c >= 40
                                               ? Motor_Group_StopTemperature
                                               : Motor_Group_StopInvalidState;
      float detail = state->motor_error != 0U
                         ? (float)state->motor_error
                         : state->temperature_c >= 40
                               ? (float)state->temperature_c
                               : (float)(now_ms - state->timestamp);
      /* Invalid live feedback is a real safety fault, not merely a rejected
       * pending target, so it still revokes an active hold. */
      Motor_GroupControl_StopWithContext(reason, (int8_t)i, detail);
      return 0;
    }

    float zero_position = aligned_zero(state->rotor_position,
                                       state->zero_rotor_position);
    arm_positions[i] = state->rotor_position;
    zero_positions[i] = zero_position;
    /* Non-zero synchronized tests must begin within the approved mechanical
     * zero neighborhood.  Returning to zero remains allowed from either +1
     * or +2 rad. */
    if (fabsf(target_offset) > 0.0001f &&
        fabsf(zero_position - arm_positions[i]) >
            GROUP_OFFSET_START_ZERO_RAD) {
      if (active_retarget == 0U)
        Motor_GroupControl_StopWithContext(
            Motor_Group_StopInvalidState, (int8_t)i,
            zero_position - arm_positions[i]);
      return 0;
    }
    new_targets[i] = zero_position + target_offset;
    float delta = new_targets[i] - arm_positions[i];
    float max_target_delta = fabsf(target_offset) <= 0.0001f
                                 ? GROUP_ZERO_RETURN_EXCURSION_RAD
                                 : GROUP_MAX_TARGET_DELTA_RAD;
    if (!isfinite(new_targets[i]) || fabsf(delta) > max_target_delta) {
      if (active_retarget == 0U)
        Motor_GroupControl_StopWithContext(Motor_Group_StopPositionLimit,
                                           (int8_t)i, delta);
      return 0;
    }
  }

  if (active_retarget != 0U) {
    /* Stage only.  The executing target, ramp, PID state, integrator and
     * authorized torques remain untouched until START commits this plan. */
    for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
      channels[i].pending_arm_position = arm_positions[i];
      channels[i].pending_target_position = new_targets[i];
    }
    pending_group_profile = profile;
    pending_group_target_offset = target_offsets[0];
    group_mode = Motor_Group_ActivePending;
    stop_reason = Motor_Group_StopNone;
    group_ready = 1U;
    return 1;
  }

  Motor_GroupControl_Init();
  group_profile = profile;
  group_target_offset = target_offsets[0];
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    Motor_GroupChannel *channel = &channels[i];
    channel->arm_position = arm_positions[i];
    channel->zero_position = zero_positions[i];
    channel->target_position = new_targets[i];
    channel->ramped_target = channel->arm_position;
    channel->actual_position = channel->arm_position;
    channel->position_error = channel->target_position - channel->arm_position;
    channel->diagnostic_position_min = channel->arm_position;
    channel->diagnostic_position_max = channel->arm_position;
  }

  group_mode = Motor_Group_Armed;
  stop_reason = Motor_Group_StopNone;
  group_ready = 1U;
  return 1;
}

int Motor_GroupControl_Start(uint32_t now_ms)
{
  (void)now_ms;
  if (group_mode == Motor_Group_ActivePending && group_ready != 0U) {
    for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
      Motor_GroupChannel *channel = &channels[i];
      channel->arm_position = channel->pending_arm_position;
      channel->target_position = channel->pending_target_position;
      channel->position_error = channel->ramped_target -
                                channel->actual_position;
      channel->diagnostic_position_min = channel->actual_position;
      channel->diagnostic_position_max = channel->actual_position;
      channel->diagnostic_max_abs_velocity = 0.0f;
      channel->diagnostic_max_abs_torque = 0.0f;
    }
    group_profile = pending_group_profile;
    group_target_offset = pending_group_target_offset;
    group_mode = Motor_Group_Active;
    stop_reason = Motor_Group_StopNone;
    group_ready = 0U;
    return 1;
  }

  if (group_mode != Motor_Group_Armed || !group_ready) return 0;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    channels[i].previous_feedback_timestamp = 0U;
    channels[i].previous_position_error = 0.0f;
    channels[i].previous_speed_error = 0.0f;
    channels[i].integral = 0.0f;
    channels[i].torque = 0.0f;
    channels[i].diagnostic_position_min = channels[i].actual_position;
    channels[i].diagnostic_position_max = channels[i].actual_position;
    channels[i].diagnostic_max_abs_velocity = 0.0f;
    channels[i].diagnostic_max_abs_torque = 0.0f;
  }
  group_mode = Motor_Group_Active;
  stop_reason = Motor_Group_StopNone;
  group_ready = 0U;
  return 1;
}

int Motor_GroupControl_ConfigureGait(const Motor_GroupGaitLimits *limits)
{
  if (limits == NULL || group_mode != Motor_Group_Active ||
      group_profile != Motor_Group_ProfileUniformOffset ||
      fabsf(group_target_offset) > 0.0001f ||
      !isfinite(limits->max_zero_excursion_rad) ||
      limits->max_zero_excursion_rad <
          GROUP_GAIT_CONFIG_MIN_EXCURSION_RAD ||
      limits->max_zero_excursion_rad >
          GROUP_GAIT_CONFIG_MAX_EXCURSION_RAD)
    return 0;

  gait_limits = *limits;
  return 1;
}

int Motor_GroupControl_SetGaitTargets(const float target_positions[8])
{
  if (target_positions == NULL || group_mode != Motor_Group_Active)
    return 0;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    float target = target_positions[i];
    if (!isfinite(target) ||
        fabsf(target - channels[i].zero_position) >
            gait_limits.max_zero_excursion_rad) {
      Motor_GroupControl_StopWithContext(Motor_Group_StopPositionLimit,
                                         (int8_t)i,
                                         target - channels[i].zero_position);
      return 0;
    }
  }
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i)
    channels[i].target_position = target_positions[i];
  group_profile = Motor_Group_ProfileGait;
  group_target_offset = 0.0f;
  return 1;
}

int Motor_GroupControl_ReturnGaitToZero(void)
{
  if (group_mode != Motor_Group_Active ||
      group_profile != Motor_Group_ProfileGait)
    return 0;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i)
    channels[i].target_position = channels[i].zero_position;
  return 1;
}

int Motor_GroupControl_FinishGaitHold(void)
{
  if (group_mode != Motor_Group_Active ||
      group_profile != Motor_Group_ProfileGait)
    return 0;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    /* Loaded standing produces a real steady-state deflection.  Finishing a
     * gait only changes controller profile while keeping the same mechanical
     * zero target, so unloaded feedback proximity must not gate this state
     * transition. */
    channels[i].target_position = channels[i].zero_position;
    channels[i].ramped_target = channels[i].zero_position;
    channels[i].integral = 0.0f;
  }
  group_profile = Motor_Group_ProfileUniformOffset;
  group_target_offset = 0.0f;
  return 1;
}

void Motor_GroupControl_Update(uint8_t motor_index,
                               float rotor_position,
                               float rotor_velocity,
                               uint32_t feedback_timestamp)
{
  if ((group_mode != Motor_Group_Active &&
       group_mode != Motor_Group_ActivePending) ||
      motor_index >= GROUP_MOTOR_COUNT)
    return;
  if (!isfinite(rotor_position) || !isfinite(rotor_velocity) ||
      feedback_timestamp == 0U) {
    Motor_GroupControl_StopWithContext(Motor_Group_StopController,
                                       (int8_t)motor_index, 0.0f);
    return;
  }

  Motor_GroupChannel *channel = &channels[motor_index];
  channel->actual_position = rotor_position;
  channel->actual_velocity = rotor_velocity;
  if (rotor_position < channel->diagnostic_position_min)
    channel->diagnostic_position_min = rotor_position;
  if (rotor_position > channel->diagnostic_position_max)
    channel->diagnostic_position_max = rotor_position;
  if (fabsf(rotor_velocity) > channel->diagnostic_max_abs_velocity)
    channel->diagnostic_max_abs_velocity = fabsf(rotor_velocity);
  /* Guard the absolute mechanical-zero neighborhood, not displacement from
   * the arm instant.  A loaded zero pose may deflect by as much as 0.5 rotor
   * rad; it must still be able to target +2 rad without falsely tripping a
   * 2.1 rad arm-relative excursion check. */
  float zero_excursion_limit =
      group_profile == Motor_Group_ProfileGait
          ? gait_limits.max_zero_excursion_rad
          : group_profile == Motor_Group_ProfileUniformOffset &&
                    fabsf(group_target_offset) <= 0.0001f
                ? GROUP_ZERO_RETURN_EXCURSION_RAD
                : GROUP_MAX_ZERO_EXCURSION_RAD;
  if (fabsf(rotor_position - channel->zero_position) >
      zero_excursion_limit) {
    Motor_GroupControl_StopWithContext(
        Motor_Group_StopPositionLimit, (int8_t)motor_index,
        rotor_position - channel->zero_position);
    return;
  }

  if (channel->previous_feedback_timestamp == 0U) {
    channel->previous_feedback_timestamp = feedback_timestamp;
    return;
  }
  if (feedback_timestamp == channel->previous_feedback_timestamp) return;

  float dt = (float)(feedback_timestamp - channel->previous_feedback_timestamp) *
             0.001f;
  channel->previous_feedback_timestamp = feedback_timestamp;
  if (!isfinite(dt) || dt < GROUP_MIN_DT_S || dt > GROUP_MAX_DT_S) {
    Motor_GroupControl_StopWithContext(Motor_Group_StopController,
                                       (int8_t)motor_index, dt * 1000.0f);
    return;
  }

  float ramp_velocity = 0.0f;
  if (group_profile == Motor_Group_ProfileGait) {
    /* Software_Ref feeds the generated gait target directly into the angle
     * PID.  It has no extra target-speed ramp and no measured-speed or
     * tracking-error abort threshold. */
    channel->ramped_target = channel->target_position;
  } else {
    float remaining = channel->target_position - channel->ramped_target;
    float target_speed = group_profile == Motor_Group_ProfileStandPose
                             ? GROUP_STAND_TARGET_SPEED_RAD_S
                             : GROUP_TARGET_SPEED_RAD_S;
    float target_step = clamp_symmetric(remaining, target_speed * dt);
    channel->ramped_target += target_step;
    ramp_velocity = target_step / dt;
  }

  float previous_position_error = channel->previous_position_error;
  channel->position_error = channel->ramped_target - rotor_position;
  channel->previous_position_error = channel->position_error;
  float angle_out_limit = group_profile == Motor_Group_ProfileGait
                              ? GROUP_GAIT_ANGLE_OUT_MAX_RAD_S
                              : GROUP_SPEED_TARGET_MAX_RAD_S;
  float speed_target = clamp_symmetric(
      ramp_velocity + GROUP_POSITION_KP * channel->position_error +
          GROUP_POSITION_KD * (channel->position_error - previous_position_error),
      angle_out_limit);
  float speed_error = speed_target - rotor_velocity;
  float speed_delta = speed_error - channel->previous_speed_error;
  channel->previous_speed_error = speed_error;
  channel->integral = clamp_symmetric(
      channel->integral + GROUP_SPEED_KI * speed_error,
      GROUP_SPEED_INTEGRAL_MAX);
  float hold_torque = 0.0f;
  if (group_profile != Motor_Group_ProfileGait &&
      fabsf(channel->target_position - channel->ramped_target) <=
      GROUP_HOLD_ENTRY_ERROR_RAD) {
    /* The reference cascade has only 0.359 N.m/rad of immediate static
     * stiffness (35.9 * 0.01); its integrator supplies load torque later.
     * Add bounded direct stiffness only after the motion ramp has completed
     * so an external displacement receives an immediate restoring response
     * while the existing cascade continues to provide damping/integration. */
    hold_torque = GROUP_HOLD_POSITION_KP_NM_RAD *
                  (channel->target_position - rotor_position);
  }
  float torque_limit = group_profile == Motor_Group_ProfileGait
                           ? GROUP_GAIT_TORQUE_MAX_NM
                           : GROUP_TORQUE_MAX_NM;
  channel->torque = clamp_symmetric(
      GROUP_SPEED_KP * speed_error + channel->integral +
          GROUP_SPEED_KD * speed_delta + hold_torque,
      torque_limit);
  if (fabsf(channel->torque) > channel->diagnostic_max_abs_torque)
    channel->diagnostic_max_abs_torque = fabsf(channel->torque);

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
  return (group_mode == Motor_Group_Active ||
          group_mode == Motor_Group_ActivePending) ? 1U : 0U;
}

int Motor_GroupControl_GetAuthorizedTorque(uint8_t motor_index, float *torque)
{
  if (torque == NULL) return 0;
  *torque = 0.0f;
  if ((group_mode != Motor_Group_Active &&
       group_mode != Motor_Group_ActivePending) ||
      motor_index >= GROUP_MOTOR_COUNT)
    return 0;
  float torque_limit = group_profile == Motor_Group_ProfileGait
                           ? GROUP_GAIT_TORQUE_MAX_NM
                           : GROUP_TORQUE_MAX_NM;
  if (!isfinite(channels[motor_index].torque) ||
      fabsf(channels[motor_index].torque) > torque_limit) {
    Motor_GroupControl_StopWithContext(Motor_Group_StopController,
                                       (int8_t)motor_index,
                                       channels[motor_index].torque);
    return 0;
  }
  *torque = channels[motor_index].torque;
  return 1;
}

void Motor_GroupControl_GetSnapshot(Motor_GroupSnapshot *snapshot)
{
  if (snapshot == NULL) return;
  memset(snapshot, 0, sizeof(*snapshot));
  uint8_t pending = group_mode == Motor_Group_ActivePending ? 1U : 0U;
  snapshot->mode = group_mode;
  snapshot->profile = pending != 0U ? pending_group_profile : group_profile;
  snapshot->reason = stop_reason;
  snapshot->ready = group_ready;
  snapshot->all_at_zero = group_mode == Motor_Group_Active ? 1U : 0U;
  snapshot->stop_motor_index = stop_motor_index;
  snapshot->stop_detail = stop_detail;
  snapshot->stop_sequence = stop_sequence;
  snapshot->target_offset = pending != 0U ? pending_group_target_offset
                                          : group_target_offset;
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    snapshot->arm_position[i] = pending != 0U
                                    ? channels[i].pending_arm_position
                                    : channels[i].arm_position;
    snapshot->target_position[i] = pending != 0U
                                       ? channels[i].pending_target_position
                                       : channels[i].target_position;
    snapshot->actual_position[i] = channels[i].actual_position;
    snapshot->position_error[i] = snapshot->target_position[i] -
                                  channels[i].actual_position;
    if (fabsf(snapshot->position_error[i]) > GROUP_TARGET_ERROR_RAD)
      snapshot->all_at_zero = 0U;
  }
}

void Motor_GroupControl_GetDiagnostics(Motor_GroupDiagnostics *diagnostics,
                                       uint8_t reset_window)
{
  if (diagnostics == NULL) return;
  memset(diagnostics, 0, sizeof(*diagnostics));
  for (uint8_t i = 0U; i < GROUP_MOTOR_COUNT; ++i) {
    Motor_GroupChannel *channel = &channels[i];
    diagnostics->position_peak_to_peak[i] =
        channel->diagnostic_position_max - channel->diagnostic_position_min;
    diagnostics->max_abs_velocity[i] = channel->diagnostic_max_abs_velocity;
    diagnostics->max_abs_torque[i] = channel->diagnostic_max_abs_torque;
    if (reset_window != 0U) {
      channel->diagnostic_position_min = channel->actual_position;
      channel->diagnostic_position_max = channel->actual_position;
      channel->diagnostic_max_abs_velocity = 0.0f;
      channel->diagnostic_max_abs_torque = 0.0f;
    }
  }
}
