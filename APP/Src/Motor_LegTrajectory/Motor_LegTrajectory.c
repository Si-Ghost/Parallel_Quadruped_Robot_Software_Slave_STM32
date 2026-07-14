#include "Motor_LegTrajectory.h"

#include <math.h>
#include <string.h>

#define LEG_TRAJ_MOTOR_COUNT             2U
#define LEG_TRAJ_REDUCTION_RATIO         6.33f
#define LEG_TRAJ_TWO_PI                  6.28318530717958647692f
#define LEG_TRAJ_MIN_DT_S                0.0002f
#define LEG_TRAJ_MAX_DT_S                0.0100f
#define LEG_TRAJ_OVERSPEED_SAMPLES       3U
#define LEG_TRAJ_TRACKING_ERROR_SAMPLES  3U

typedef struct
{
  float arm_position;
  float zero_position;
  float direction;
  float arm_joint_position;
  float base_joint_position;
  int8_t arm_temperature_c;
  uint8_t arm_zero_checked;
  uint32_t arm_feedback_age_ms;
  float target_position;
  float previous_target_position;
  float actual_position;
  float position_error;
  float raw_velocity;
  float target_velocity;
  float speed_target;
  float speed_error;
  float previous_position_error;
  float previous_speed_error;
  float p_term;
  float i_term;
  float d_term;
  float torque;
  uint8_t torque_limited;
  uint8_t overspeed_count;
  uint8_t tracking_error_count;
  uint32_t feedback_count;
  uint32_t torque_limit_count;
  float peak_abs_target_velocity;
  float peak_abs_actual_velocity;
  float peak_abs_position_error;
  float peak_abs_torque;
  float min_dt_ms;
  float max_dt_ms;
  uint32_t previous_feedback_timestamp;
} Motor_LegTrajectoryChannel;

typedef struct
{
  Motor_LegTrajectoryMode mode;
  Motor_LegTrajectoryStopReason reason;
  uint8_t dry_run;
  uint8_t dry_run_passed;
  uint8_t trajectory_complete;
  int8_t stop_motor_index;
  float stop_detail;
  uint32_t stop_sequence;
  uint32_t plan_start_ms;
  uint32_t elapsed_ms;
  uint32_t last_target_ms;
  float phase;
  Leg_PointTypeDef base_foot;
  Leg_PointTypeDef target_foot;
  float peak_target_delta[LEG_TRAJ_MOTOR_COUNT];
  Motor_LegTrajectoryChannel channel[LEG_TRAJ_MOTOR_COUNT];
} Motor_LegTrajectoryContext;

static Motor_LegTrajectoryContext control;

static float clamp_symmetric(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

static float max_abs(float current, float value)
{
  float magnitude = fabsf(value);
  return magnitude > current ? magnitude : current;
}

static float normalized_direction(float direction)
{
  return direction < 0.0f ? -1.0f : 1.0f;
}

static uint8_t running_mode(void)
{
  return (control.mode == Motor_LegTrajectory_DryRun ||
          control.mode == Motor_LegTrajectory_Active) ? 1U : 0U;
}

void Motor_LegTrajectory_Init(void)
{
  memset(&control, 0, sizeof(control));
  control.mode = Motor_LegTrajectory_Disabled;
  control.stop_motor_index = -1;
}

static int compute_rotor_targets(const Leg_PointTypeDef *foot,
                                 float rotor_targets[LEG_TRAJ_MOTOR_COUNT])
{
  Leg_JointAnglesTypeDef angles;
  if (foot == NULL || rotor_targets == NULL ||
      !Leg_Kinematics_Inverse(foot, &angles)) return 0;

  const float joint_targets[LEG_TRAJ_MOTOR_COUNT] = {
      angles.theta1, angles.theta2};
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    Motor_LegTrajectoryChannel *channel = &control.channel[motor];
    /* The raw target is anchored to the arm-time multi-turn position.  Only
     * the continuous IK joint delta is applied, so no wrap or synthetic turn
     * accumulation can move the command to another rotor revolution. */
    rotor_targets[motor] = channel->arm_position + channel->direction *
        (joint_targets[motor] - channel->base_joint_position) *
        LEG_TRAJ_REDUCTION_RATIO;
    if (!isfinite(rotor_targets[motor]) ||
        fabsf(rotor_targets[motor] - channel->arm_position) >
            MOTOR_LEG_TRAJECTORY_TARGET_DELTA_MAX)
      return 0;
  }
  return 1;
}

int Motor_LegTrajectory_Arm(const Motor_StateSnapshotTypeDef states[2],
                            uint32_t now_ms)
{
  if (states == NULL) return 0;
  uint8_t dry_run_passed = control.dry_run_passed;
  uint32_t stop_sequence = control.stop_sequence;
  memset(&control, 0, sizeof(control));
  control.dry_run_passed = dry_run_passed;
  control.stop_sequence = stop_sequence;
  control.stop_motor_index = -1;

  Leg_JointAnglesTypeDef current_angles = {
      .theta1 = states[0].joint_position,
      .theta2 = states[1].joint_position,
  };
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    const Motor_StateSnapshotTypeDef *state = &states[motor];
    if (!state->online || !state->angle_valid || state->timestamp == 0U ||
        (now_ms - state->timestamp) > 10U ||
        !isfinite(state->rotor_position) ||
        !isfinite(state->joint_position) || state->motor_error != 0U ||
        state->temperature_c >= 40) {
      control.mode = Motor_LegTrajectory_Stopped;
      control.reason = state->motor_error != 0U
                           ? Motor_LegTrajectory_StopMotorFault
                           : state->temperature_c >= 40
                                 ? Motor_LegTrajectory_StopTemperature
                                 : Motor_LegTrajectory_StopOffline;
      control.stop_motor_index =
          (int8_t)(MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX + motor);
      control.stop_detail = state->motor_error != 0U
                                ? (float)state->motor_error
                                : state->temperature_c >= 40
                                      ? (float)state->temperature_c
                                      : (float)(now_ms - state->timestamp);
      ++control.stop_sequence;
      return 0;
    }
    Motor_LegTrajectoryChannel *channel = &control.channel[motor];
    channel->arm_position = state->rotor_position;
    channel->zero_position = state->zero_rotor_position;
    channel->direction = normalized_direction(state->direction);
    channel->arm_joint_position = state->joint_position;
    channel->arm_temperature_c = state->temperature_c;
    channel->arm_zero_checked = state->zero_checked;
    channel->arm_feedback_age_ms = now_ms - state->timestamp;
    channel->actual_position = state->rotor_position;
    channel->target_position = state->rotor_position;
    channel->previous_target_position = state->rotor_position;
  }

  if (!Leg_Kinematics_Forward(&current_angles, &control.base_foot)) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopIk, -1, 0.0f);
    return 0;
  }

  Leg_JointAnglesTypeDef base_ik;
  if (!Leg_Kinematics_Inverse(&control.base_foot, &base_ik)) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopIk, -1, 1.0f);
    return 0;
  }
  control.channel[0].base_joint_position = base_ik.theta1;
  control.channel[1].base_joint_position = base_ik.theta2;

  Leg_PointTypeDef peak_foot = {
      .x = control.base_foot.x,
      .y = control.base_foot.y - MOTOR_LEG_TRAJECTORY_LIFT_MM,
  };
  float peak_targets[LEG_TRAJ_MOTOR_COUNT];
  if (!compute_rotor_targets(&peak_foot, peak_targets)) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopPosition, -1,
                             peak_foot.y);
    return 0;
  }
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor)
    control.peak_target_delta[motor] =
        peak_targets[motor] - control.channel[motor].arm_position;

  control.target_foot = control.base_foot;
  control.mode = Motor_LegTrajectory_Armed;
  control.reason = Motor_LegTrajectory_StopNone;
  return 1;
}

int Motor_LegTrajectory_Start(uint8_t dry_run, uint32_t now_ms)
{
  if (control.mode != Motor_LegTrajectory_Armed || dry_run > 1U ||
      (dry_run == 0U && MOTOR_LEG_TRAJECTORY_ACTIVE_ENABLED == 0U) ||
      (dry_run == 0U && control.dry_run_passed == 0U)) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopInvalidCommand, -1,
                             (float)dry_run);
    return 0;
  }

  control.mode = dry_run != 0U ? Motor_LegTrajectory_DryRun
                               : Motor_LegTrajectory_Active;
  control.reason = Motor_LegTrajectory_StopNone;
  control.dry_run = dry_run;
  control.trajectory_complete = 0U;
  control.plan_start_ms = now_ms;
  control.elapsed_ms = 0U;
  control.last_target_ms = now_ms;
  control.phase = 0.0f;
  control.target_foot = control.base_foot;
  control.stop_motor_index = -1;
  control.stop_detail = 0.0f;
  control.dry_run_passed = 0U;
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    Motor_LegTrajectoryChannel *channel = &control.channel[motor];
    channel->target_position = channel->arm_position;
    channel->previous_target_position = channel->arm_position;
    channel->position_error = 0.0f;
    channel->target_velocity = 0.0f;
    channel->speed_target = 0.0f;
    channel->speed_error = 0.0f;
    channel->previous_position_error = 0.0f;
    channel->previous_speed_error = 0.0f;
    channel->p_term = 0.0f;
    channel->i_term = 0.0f;
    channel->d_term = 0.0f;
    channel->torque = 0.0f;
    channel->torque_limited = 0U;
    channel->overspeed_count = 0U;
    channel->tracking_error_count = 0U;
    channel->feedback_count = 0U;
    channel->torque_limit_count = 0U;
    channel->peak_abs_target_velocity = 0.0f;
    channel->peak_abs_actual_velocity = 0.0f;
    channel->peak_abs_position_error = 0.0f;
    channel->peak_abs_torque = 0.0f;
    channel->min_dt_ms = 0.0f;
    channel->max_dt_ms = 0.0f;
    channel->previous_feedback_timestamp = 0U;
  }
  return 1;
}

static int update_trajectory_targets(uint32_t now_ms)
{
  if (running_mode() == 0U) return 0;
  control.elapsed_ms = now_ms - control.plan_start_ms;

  float shape = 0.0f;
  if (control.elapsed_ms >= MOTOR_LEG_TRAJECTORY_PERIOD_MS) {
    control.phase = 1.0f;
    control.trajectory_complete = 1U;
  } else {
    control.phase = (float)control.elapsed_ms /
                    (float)MOTOR_LEG_TRAJECTORY_PERIOD_MS;
    shape = 0.5f - 0.5f * cosf(LEG_TRAJ_TWO_PI * control.phase);
    control.trajectory_complete = 0U;
  }

  control.target_foot.x = control.base_foot.x;
  control.target_foot.y = control.base_foot.y -
                          MOTOR_LEG_TRAJECTORY_LIFT_MM * shape;
  float targets[LEG_TRAJ_MOTOR_COUNT];
  if (!compute_rotor_targets(&control.target_foot, targets)) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopIk, -1,
                             control.target_foot.y);
    return 0;
  }

  uint32_t target_dt_ms = now_ms - control.last_target_ms;
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    Motor_LegTrajectoryChannel *channel = &control.channel[motor];
    channel->target_position = targets[motor];
    if (target_dt_ms != 0U) {
      float target_dt_s = (float)target_dt_ms * 0.001f;
      channel->target_velocity = clamp_symmetric(
          (channel->target_position - channel->previous_target_position) /
              target_dt_s,
          MOTOR_LEG_TRAJECTORY_TARGET_SPEED_MAX);
      channel->peak_abs_target_velocity = max_abs(
          channel->peak_abs_target_velocity, channel->target_velocity);
      channel->previous_target_position = channel->target_position;
    }
  }
  if (target_dt_ms != 0U) control.last_target_ms = now_ms;
  return 1;
}

void Motor_LegTrajectory_Update(uint8_t motor_index,
                                float rotor_position,
                                float rotor_velocity,
                                uint32_t feedback_timestamp,
                                uint32_t now_ms)
{
  if (running_mode() == 0U ||
      motor_index < MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX ||
      motor_index >= MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX +
                         LEG_TRAJ_MOTOR_COUNT)
    return;
  if (!isfinite(rotor_position) || !isfinite(rotor_velocity) ||
      feedback_timestamp == 0U || !update_trajectory_targets(now_ms)) {
    if (control.mode != Motor_LegTrajectory_Stopped)
      Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopController,
                               (int8_t)motor_index, 0.0f);
    return;
  }

  uint8_t local = motor_index - MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX;
  Motor_LegTrajectoryChannel *channel = &control.channel[local];
  channel->actual_position = rotor_position;
  channel->raw_velocity = rotor_velocity;
  channel->position_error = channel->target_position - rotor_position;
  ++channel->feedback_count;
  channel->peak_abs_actual_velocity = max_abs(
      channel->peak_abs_actual_velocity, rotor_velocity);
  channel->peak_abs_position_error = max_abs(
      channel->peak_abs_position_error, channel->position_error);

  if (control.mode == Motor_LegTrajectory_Active &&
      fabsf(channel->position_error) >
          MOTOR_LEG_TRAJECTORY_TRACKING_ERROR_MAX) {
    if (channel->tracking_error_count < 255U)
      ++channel->tracking_error_count;
    if (channel->tracking_error_count >=
        LEG_TRAJ_TRACKING_ERROR_SAMPLES) {
      Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopPosition,
                               (int8_t)motor_index,
                               channel->position_error);
      return;
    }
  } else {
    channel->tracking_error_count = 0U;
  }

  if (fabsf(rotor_position - channel->arm_position) >
      MOTOR_LEG_TRAJECTORY_POSITION_MAX) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopPosition,
                             (int8_t)motor_index,
                             rotor_position - channel->arm_position);
    return;
  }
  if (fabsf(rotor_velocity) > MOTOR_LEG_TRAJECTORY_ACTUAL_SPEED_MAX) {
    if (channel->overspeed_count < 255U) ++channel->overspeed_count;
    if (channel->overspeed_count >= LEG_TRAJ_OVERSPEED_SAMPLES) {
      Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopVelocity,
                               (int8_t)motor_index, rotor_velocity);
      return;
    }
  } else {
    channel->overspeed_count = 0U;
  }

  if (channel->previous_feedback_timestamp == 0U) {
    channel->previous_feedback_timestamp = feedback_timestamp;
    channel->previous_position_error = channel->position_error;
    return;
  }
  if (feedback_timestamp == channel->previous_feedback_timestamp) return;
  float dt = (float)(feedback_timestamp -
                     channel->previous_feedback_timestamp) * 0.001f;
  channel->previous_feedback_timestamp = feedback_timestamp;
  if (!isfinite(dt) || dt < LEG_TRAJ_MIN_DT_S || dt > LEG_TRAJ_MAX_DT_S) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopController,
                             (int8_t)motor_index, dt * 1000.0f);
    return;
  }
  float dt_ms = dt * 1000.0f;
  if (channel->min_dt_ms == 0.0f || dt_ms < channel->min_dt_ms)
    channel->min_dt_ms = dt_ms;
  if (dt_ms > channel->max_dt_ms) channel->max_dt_ms = dt_ms;

  float position_delta = channel->position_error -
                         channel->previous_position_error;
  channel->previous_position_error = channel->position_error;
  channel->speed_target = clamp_symmetric(
      channel->target_velocity + MOTOR_LEG_TRAJECTORY_POSITION_KP *
          channel->position_error +
          MOTOR_LEG_TRAJECTORY_POSITION_KD * position_delta,
      MOTOR_LEG_TRAJECTORY_TARGET_SPEED_MAX);
  channel->speed_error = channel->speed_target - rotor_velocity;
  float speed_delta = channel->speed_error - channel->previous_speed_error;
  channel->previous_speed_error = channel->speed_error;
  channel->p_term = MOTOR_LEG_TRAJECTORY_SPEED_KP * channel->speed_error;
  channel->i_term = clamp_symmetric(
      channel->i_term + MOTOR_LEG_TRAJECTORY_SPEED_KI *
                            channel->speed_error,
      MOTOR_LEG_TRAJECTORY_INTEGRAL_MAX);
  channel->d_term = MOTOR_LEG_TRAJECTORY_SPEED_KD * speed_delta;
  float calculated_torque = channel->p_term + channel->i_term +
                            channel->d_term;
  channel->torque = clamp_symmetric(calculated_torque,
                                    MOTOR_LEG_TRAJECTORY_TORQUE_MAX_NM);
  channel->torque_limited =
      fabsf(calculated_torque - channel->torque) > 0.000001f ? 1U : 0U;
  if (channel->torque_limited != 0U) ++channel->torque_limit_count;
  channel->peak_abs_torque = max_abs(channel->peak_abs_torque,
                                     channel->torque);
}

void Motor_LegTrajectory_Stop(Motor_LegTrajectoryStopReason reason,
                              int8_t motor_index,
                              float detail)
{
  if (reason != Motor_LegTrajectory_StopNone &&
      control.mode == Motor_LegTrajectory_Stopped)
    return;

  Motor_LegTrajectoryMode previous_mode = control.mode;
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    control.channel[motor].torque = 0.0f;
    control.channel[motor].i_term = 0.0f;
  }
  if (reason == Motor_LegTrajectory_StopNone) {
    control.mode = Motor_LegTrajectory_Disabled;
    control.reason = Motor_LegTrajectory_StopNone;
    control.stop_motor_index = -1;
    control.stop_detail = 0.0f;
    return;
  }

  if (reason == Motor_LegTrajectory_StopComplete &&
      previous_mode == Motor_LegTrajectory_DryRun)
    control.dry_run_passed = 1U;
  control.mode = Motor_LegTrajectory_Stopped;
  control.reason = reason;
  control.stop_motor_index = motor_index;
  control.stop_detail = isfinite(detail) ? detail : 0.0f;
  ++control.stop_sequence;
}

uint8_t Motor_LegTrajectory_IsRunning(void)
{
  return running_mode();
}

int Motor_LegTrajectory_GetAuthorizedTorque(uint8_t motor_index,
                                            float *torque)
{
  if (torque == NULL) return 0;
  *torque = 0.0f;
  if (control.mode != Motor_LegTrajectory_Active ||
      motor_index < MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX ||
      motor_index >= MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX +
                         LEG_TRAJ_MOTOR_COUNT)
    return 0;
  uint8_t local = motor_index - MOTOR_LEG_TRAJECTORY_FIRST_MOTOR_INDEX;
  if (!isfinite(control.channel[local].torque) ||
      fabsf(control.channel[local].torque) >
          MOTOR_LEG_TRAJECTORY_TORQUE_MAX_NM) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopController,
                             (int8_t)motor_index,
                             control.channel[local].torque);
    return 0;
  }
  *torque = control.channel[local].torque;
  return 1;
}

void Motor_LegTrajectory_GetSnapshot(Motor_LegTrajectorySnapshot *snapshot)
{
  if (snapshot == NULL) return;
  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->mode = control.mode;
  snapshot->reason = control.reason;
  snapshot->dry_run = control.dry_run;
  snapshot->dry_run_passed = control.dry_run_passed;
  snapshot->trajectory_complete = control.trajectory_complete;
  snapshot->stop_motor_index = control.stop_motor_index;
  snapshot->stop_detail = control.stop_detail;
  snapshot->stop_sequence = control.stop_sequence;
  snapshot->elapsed_ms = control.elapsed_ms;
  snapshot->phase = control.phase;
  snapshot->base_foot = control.base_foot;
  snapshot->target_foot = control.target_foot;
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    const Motor_LegTrajectoryChannel *channel = &control.channel[motor];
    snapshot->peak_target_delta[motor] = control.peak_target_delta[motor];
    snapshot->arm_position[motor] = channel->arm_position;
    snapshot->zero_position[motor] = channel->zero_position;
    snapshot->direction[motor] = channel->direction;
    snapshot->arm_joint_position[motor] = channel->arm_joint_position;
    snapshot->base_joint_position[motor] = channel->base_joint_position;
    snapshot->arm_temperature_c[motor] = channel->arm_temperature_c;
    snapshot->arm_zero_checked[motor] = channel->arm_zero_checked;
    snapshot->arm_feedback_age_ms[motor] = channel->arm_feedback_age_ms;
    snapshot->target_position[motor] = channel->target_position;
    snapshot->actual_position[motor] = channel->actual_position;
    snapshot->position_error[motor] = channel->position_error;
    snapshot->raw_velocity[motor] = channel->raw_velocity;
    snapshot->target_velocity[motor] = channel->target_velocity;
    snapshot->speed_target[motor] = channel->speed_target;
    snapshot->speed_error[motor] = channel->speed_error;
    snapshot->p_term[motor] = channel->p_term;
    snapshot->i_term[motor] = channel->i_term;
    snapshot->d_term[motor] = channel->d_term;
    snapshot->torque[motor] = channel->torque;
    snapshot->torque_limited[motor] = channel->torque_limited;
    snapshot->overspeed_count[motor] = channel->overspeed_count;
    snapshot->tracking_error_count[motor] =
        channel->tracking_error_count;
    snapshot->feedback_count[motor] = channel->feedback_count;
    snapshot->torque_limit_count[motor] = channel->torque_limit_count;
    snapshot->peak_abs_target_velocity[motor] =
        channel->peak_abs_target_velocity;
    snapshot->peak_abs_actual_velocity[motor] =
        channel->peak_abs_actual_velocity;
    snapshot->peak_abs_position_error[motor] =
        channel->peak_abs_position_error;
    snapshot->peak_abs_torque[motor] = channel->peak_abs_torque;
    snapshot->min_dt_ms[motor] = channel->min_dt_ms;
    snapshot->max_dt_ms[motor] = channel->max_dt_ms;
  }
}
