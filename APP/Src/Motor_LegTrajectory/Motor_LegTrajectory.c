#include "Motor_LegTrajectory.h"

#include <math.h>
#include <string.h>

#define LEG_TRAJ_MOTOR_COUNT             2U
#define LEG_TRAJ_REDUCTION_RATIO         6.33f
#define LEG_TRAJ_TWO_PI                  6.28318530717958647692f
#define LEG_TRAJ_MIN_DT_S                0.0002f
#define LEG_TRAJ_MAX_DT_S                0.0100f
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
  uint8_t hard_overspeed_count;
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
  uint8_t approved_plan_available;
  uint8_t dry_run_plan_match;
  uint8_t trajectory_complete;
  uint8_t hold_current_position;
  uint8_t leg_index;
  uint8_t first_motor_index;
  Motor_LegTrajectoryProfile profile;
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
  float plan_arm_position_diff[LEG_TRAJ_MOTOR_COUNT];
  Leg_PointTypeDef plan_base_foot_diff;
  float plan_target_delta_diff[LEG_TRAJ_MOTOR_COUNT];
  Motor_LegTrajectoryChannel channel[LEG_TRAJ_MOTOR_COUNT];
} Motor_LegTrajectoryContext;

typedef struct
{
  uint8_t valid;
  Motor_LegTrajectoryProfile profile;
  Leg_PointTypeDef base_foot;
  float arm_position[LEG_TRAJ_MOTOR_COUNT];
  float peak_target_delta[LEG_TRAJ_MOTOR_COUNT];
} Motor_LegTrajectoryApprovedPlan;

static Motor_LegTrajectoryContext control;
static Motor_LegTrajectoryApprovedPlan approved_plans[
    MOTOR_LEG_TRAJECTORY_LEG_COUNT]
    [MOTOR_LEG_TRAJECTORY_LEVEL_MAX - MOTOR_LEG_TRAJECTORY_LEVEL_MIN + 1U];

static const Motor_LegTrajectoryProfile profiles[] = {
    {
        .level = 1U,
        .lift_mm = 5.0f,
        .period_ms = 4000U,
        .settle_ms = 1000U,
        .duration_ms = 5000U,
        .target_speed_max = 2.0f,
        .actual_speed_max = 4.50f,
        .hard_speed_max = 8.00f,
        .position_max = 0.75f,
        .target_delta_max = 0.55f,
        .reference_s2 = 0U,
        .reference_height_mm = 0.0f,
    },
    {
        /* The 40 mm amplitude comes from the reference project's fastest
         * 350 ms full gait cycle.  Level 2 remains the proven slow fallback;
         * Levels 3/4 shorten only the cycle without adding its +/-75 mm step. */
        .level = 2U,
        .lift_mm = 40.0f,
        .period_ms = 8000U,
        .settle_ms = 1000U,
        .duration_ms = 9000U,
        .target_speed_max = 1.10f,
        .actual_speed_max = 4.50f,
        .hard_speed_max = 8.00f,
        .position_max = 2.50f,
        .target_delta_max = 2.30f,
        .reference_s2 = 0U,
        .reference_height_mm = 0.0f,
    },
    {
        .level = 3U,
        .lift_mm = 40.0f,
        .period_ms = 4000U,
        .settle_ms = 1000U,
        .duration_ms = 5000U,
        .target_speed_max = 2.20f,
        .actual_speed_max = 6.00f,
        .hard_speed_max = 8.00f,
        .position_max = 2.50f,
        .target_delta_max = 2.30f,
        .reference_s2 = 0U,
        .reference_height_mm = 0.0f,
    },
    {
        .level = 4U,
        .lift_mm = 40.0f,
        .period_ms = 2000U,
        .settle_ms = 1000U,
        .duration_ms = 3000U,
        .target_speed_max = 4.00f,
        .actual_speed_max = 7.00f,
        .hard_speed_max = 8.00f,
        .position_max = 2.50f,
        .target_delta_max = 2.30f,
        .reference_s2 = 0U,
        .reference_height_mm = 0.0f,
    },
    {
        /* Reference S2=2: step_rate=300 ms, full cycle=600 ms,
         * leg_high=220 mm, lift=50 mm.  The single-leg no-L2 test keeps
         * the current arm-time base foot and applies only lift/cadence. */
        .level = 5U,
        .lift_mm = 50.0f,
        .period_ms = 600U,
        .settle_ms = 1000U,
        .duration_ms = 1600U,
        .target_speed_max = 18.0f,
        .actual_speed_max = 28.0f,
        .hard_speed_max = 36.0f,
        .position_max = 3.50f,
        .target_delta_max = 3.20f,
        .reference_s2 = 2U,
        .reference_height_mm = 220.0f,
    },
    {
        /* Reference S2=3: step_rate=220 ms, full cycle=440 ms,
         * leg_high=215 mm, lift=50 mm. */
        .level = 6U,
        .lift_mm = 50.0f,
        .period_ms = 440U,
        .settle_ms = 1000U,
        .duration_ms = 1440U,
        .target_speed_max = 24.0f,
        .actual_speed_max = 35.0f,
        .hard_speed_max = 45.0f,
        .position_max = 3.50f,
        .target_delta_max = 3.20f,
        .reference_s2 = 3U,
        .reference_height_mm = 215.0f,
    },
    {
        /* Reference S2=1: step_rate=175 ms, full cycle=350 ms,
         * leg_high=210 mm, lift=40 mm. */
        .level = 7U,
        .lift_mm = 40.0f,
        .period_ms = 350U,
        .settle_ms = 1000U,
        .duration_ms = 1350U,
        .target_speed_max = 24.0f,
        .actual_speed_max = 35.0f,
        .hard_speed_max = 45.0f,
        .position_max = 2.50f,
        .target_delta_max = 2.30f,
        .reference_s2 = 1U,
        .reference_height_mm = 210.0f,
    },
};

static int get_profile(uint8_t level, Motor_LegTrajectoryProfile *profile)
{
  if (profile == NULL || level < MOTOR_LEG_TRAJECTORY_LEVEL_MIN ||
      level > MOTOR_LEG_TRAJECTORY_LEVEL_MAX)
    return 0;
  *profile = profiles[level - MOTOR_LEG_TRAJECTORY_LEVEL_MIN];
  return profile->level == level;
}

static Motor_LegTrajectoryApprovedPlan *approved_plan_for_level(uint8_t leg_index,
                                                                 uint8_t level)
{
  if (leg_index >= MOTOR_LEG_TRAJECTORY_LEG_COUNT ||
      level < MOTOR_LEG_TRAJECTORY_LEVEL_MIN ||
      level > MOTOR_LEG_TRAJECTORY_LEVEL_MAX)
    return NULL;
  return &approved_plans[leg_index]
                        [level - MOTOR_LEG_TRAJECTORY_LEVEL_MIN];
}

static uint8_t profiles_match(const Motor_LegTrajectoryProfile *a,
                              const Motor_LegTrajectoryProfile *b)
{
  if (a == NULL || b == NULL) return 0U;
  return (a->level == b->level && a->lift_mm == b->lift_mm &&
          a->period_ms == b->period_ms && a->settle_ms == b->settle_ms &&
          a->duration_ms == b->duration_ms &&
          a->target_speed_max == b->target_speed_max &&
          a->actual_speed_max == b->actual_speed_max &&
          a->hard_speed_max == b->hard_speed_max &&
          a->position_max == b->position_max &&
          a->target_delta_max == b->target_delta_max &&
          a->reference_s2 == b->reference_s2 &&
          a->reference_height_mm == b->reference_height_mm) ? 1U : 0U;
}

static void evaluate_approved_plan(void)
{
  Motor_LegTrajectoryApprovedPlan *approved =
      approved_plan_for_level(control.leg_index, control.profile.level);
  control.approved_plan_available =
      approved != NULL && approved->valid != 0U ? 1U : 0U;
  control.dry_run_passed = 0U;
  control.dry_run_plan_match = 0U;
  if (control.approved_plan_available == 0U) return;

  control.plan_base_foot_diff.x =
      control.base_foot.x - approved->base_foot.x;
  control.plan_base_foot_diff.y =
      control.base_foot.y - approved->base_foot.y;
  uint8_t match = profiles_match(&control.profile, &approved->profile);
  float max_arm_diff = 0.0f;
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    control.plan_arm_position_diff[motor] =
        control.channel[motor].arm_position - approved->arm_position[motor];
    control.plan_target_delta_diff[motor] =
        control.peak_target_delta[motor] - approved->peak_target_delta[motor];
    float arm_diff_magnitude =
        fabsf(control.plan_arm_position_diff[motor]);
    if (arm_diff_magnitude > max_arm_diff)
      max_arm_diff = arm_diff_magnitude;
    if (fabsf(control.plan_arm_position_diff[motor]) >
            MOTOR_LEG_TRAJECTORY_PLAN_ARM_TOLERANCE ||
        fabsf(control.plan_target_delta_diff[motor]) >
            MOTOR_LEG_TRAJECTORY_PLAN_DELTA_TOLERANCE)
      match = 0U;
  }
  if (fabsf(control.plan_base_foot_diff.x) >
          MOTOR_LEG_TRAJECTORY_PLAN_FOOT_TOLERANCE ||
      fabsf(control.plan_base_foot_diff.y) >
          MOTOR_LEG_TRAJECTORY_PLAN_FOOT_TOLERANCE)
    match = 0U;

  control.dry_run_plan_match = match;
  control.dry_run_passed = match;
  if (match == 0U) {
    control.reason = Motor_LegTrajectory_StopPlanMismatch;
    control.stop_detail = max_arm_diff;
  }
}

static void approve_current_plan(void)
{
  Motor_LegTrajectoryApprovedPlan *approved =
      approved_plan_for_level(control.leg_index, control.profile.level);
  if (approved == NULL) return;
  memset(approved, 0, sizeof(*approved));
  approved->valid = 1U;
  approved->profile = control.profile;
  approved->base_foot = control.base_foot;
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    approved->arm_position[motor] = control.channel[motor].arm_position;
    approved->peak_target_delta[motor] = control.peak_target_delta[motor];
  }
  control.approved_plan_available = 1U;
  control.dry_run_plan_match = 1U;
  control.dry_run_passed = 1U;
  memset(control.plan_arm_position_diff, 0,
         sizeof(control.plan_arm_position_diff));
  memset(&control.plan_base_foot_diff, 0,
         sizeof(control.plan_base_foot_diff));
  memset(control.plan_target_delta_diff, 0,
         sizeof(control.plan_target_delta_diff));
}

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

static uint8_t trajectory_mode(void)
{
  return (control.mode == Motor_LegTrajectory_DryRun ||
          control.mode == Motor_LegTrajectory_Active) ? 1U : 0U;
}

static uint8_t controller_mode(void)
{
  return (trajectory_mode() != 0U ||
          control.mode == Motor_LegTrajectory_Hold) ? 1U : 0U;
}

static void reset_channel_controller(Motor_LegTrajectoryChannel *channel)
{
  if (channel == NULL) return;
  channel->target_velocity = 0.0f;
  channel->speed_target = 0.0f;
  channel->speed_error = 0.0f;
  channel->previous_position_error =
      channel->target_position - channel->actual_position;
  channel->previous_speed_error = 0.0f;
  channel->p_term = 0.0f;
  channel->i_term = 0.0f;
  channel->d_term = 0.0f;
  channel->torque = 0.0f;
  channel->torque_limited = 0U;
  channel->overspeed_count = 0U;
  channel->hard_overspeed_count = 0U;
  channel->tracking_error_count = 0U;
  channel->previous_feedback_timestamp = 0U;
}

void Motor_LegTrajectory_Init(void)
{
  memset(&control, 0, sizeof(control));
  memset(approved_plans, 0, sizeof(approved_plans));
  (void)get_profile(MOTOR_LEG_TRAJECTORY_LEVEL_MIN, &control.profile);
  control.leg_index = MOTOR_LEG_TRAJECTORY_RF_LEG_INDEX;
  control.first_motor_index =
      (uint8_t)(MOTOR_LEG_TRAJECTORY_RF_LEG_INDEX * LEG_TRAJ_MOTOR_COUNT);
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
            control.profile.target_delta_max)
      return 0;
  }
  return 1;
}

int Motor_LegTrajectory_Arm(const Motor_StateSnapshotTypeDef states[2],
                            uint8_t leg_index,
                            uint8_t level,
                            uint32_t now_ms)
{
  Motor_LegTrajectoryProfile profile;
  if (states == NULL || leg_index >= MOTOR_LEG_TRAJECTORY_LEG_COUNT ||
      level < MOTOR_LEG_TRAJECTORY_LEVEL_MIN ||
      level > MOTOR_LEG_TRAJECTORY_LEVEL_MAX ||
      !get_profile(level, &profile)) return 0;
  uint32_t stop_sequence = control.stop_sequence;
  memset(&control, 0, sizeof(control));
  control.leg_index = leg_index;
  control.first_motor_index = (uint8_t)(leg_index * LEG_TRAJ_MOTOR_COUNT);
  control.profile = profile;
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
          (int8_t)(control.first_motor_index + motor);
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
      .y = control.base_foot.y - control.profile.lift_mm,
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
  evaluate_approved_plan();
  return 1;
}

int Motor_LegTrajectory_Start(uint8_t dry_run, uint32_t now_ms)
{
  if (control.mode != Motor_LegTrajectory_Armed || dry_run > 1U ||
      (dry_run == 0U && MOTOR_LEG_TRAJECTORY_ACTIVE_ENABLED == 0U) ||
      (dry_run == 0U && MOTOR_LEG_TRAJECTORY_DRY_RUN_REQUIRED != 0U &&
       (control.dry_run_passed == 0U ||
        control.dry_run_plan_match == 0U))) {
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
  if (dry_run != 0U) {
    Motor_LegTrajectoryApprovedPlan *approved =
        approved_plan_for_level(control.leg_index, control.profile.level);
    if (approved != NULL) approved->valid = 0U;
    control.approved_plan_available = 0U;
    control.dry_run_plan_match = 0U;
    control.dry_run_passed = 0U;
  }
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
    channel->hard_overspeed_count = 0U;
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

int Motor_LegTrajectory_EnterHold(Motor_LegTrajectoryStopReason reason,
                                  uint8_t hold_current_position,
                                  float detail)
{
  if ((control.mode != Motor_LegTrajectory_Active &&
       control.mode != Motor_LegTrajectory_Hold) ||
      (reason != Motor_LegTrajectory_StopComplete &&
       reason != Motor_LegTrajectory_StopOperator &&
       reason != Motor_LegTrajectory_StopCommandLink))
    return 0;

  if (control.mode == Motor_LegTrajectory_Hold) return 1;

  /* Normal completion returns to the arm-time multi-turn targets.  An
   * operator or command-link interruption freezes the latest validated raw
   * feedback instead, preventing the remainder of the foot trajectory from
   * continuing without an operator while still preserving controlled torque. */
  for (uint8_t motor = 0U; motor < LEG_TRAJ_MOTOR_COUNT; ++motor) {
    Motor_LegTrajectoryChannel *channel = &control.channel[motor];
    channel->target_position = hold_current_position != 0U
                                   ? channel->actual_position
                                   : channel->arm_position;
    channel->previous_target_position = channel->target_position;
    channel->position_error =
        channel->target_position - channel->actual_position;
    reset_channel_controller(channel);
  }

  if (hold_current_position == 0U) {
    control.target_foot = control.base_foot;
  } else {
    Leg_JointAnglesTypeDef hold_angles = {
        .theta1 = control.channel[0].base_joint_position +
                  control.channel[0].direction *
                      (control.channel[0].target_position -
                       control.channel[0].arm_position) /
                      LEG_TRAJ_REDUCTION_RATIO,
        .theta2 = control.channel[1].base_joint_position +
                  control.channel[1].direction *
                      (control.channel[1].target_position -
                       control.channel[1].arm_position) /
                      LEG_TRAJ_REDUCTION_RATIO,
    };
    Leg_PointTypeDef hold_foot;
    if (Leg_Kinematics_Forward(&hold_angles, &hold_foot))
      control.target_foot = hold_foot;
  }

  control.mode = Motor_LegTrajectory_Hold;
  control.reason = reason;
  control.dry_run = 0U;
  control.trajectory_complete =
      reason == Motor_LegTrajectory_StopComplete ? 1U : 0U;
  control.hold_current_position = hold_current_position != 0U ? 1U : 0U;
  control.phase = reason == Motor_LegTrajectory_StopComplete ? 1.0f
                                                              : control.phase;
  control.stop_motor_index = -1;
  control.stop_detail = isfinite(detail) ? detail : 0.0f;
  ++control.stop_sequence;
  return 1;
}

static int update_trajectory_targets(uint32_t now_ms)
{
  if (trajectory_mode() == 0U) return 0;
  control.elapsed_ms = now_ms - control.plan_start_ms;

  float shape = 0.0f;
  if (control.elapsed_ms >= control.profile.period_ms) {
    control.phase = 1.0f;
    control.trajectory_complete = 1U;
  } else {
    control.phase = (float)control.elapsed_ms /
                    (float)control.profile.period_ms;
    shape = 0.5f - 0.5f * cosf(LEG_TRAJ_TWO_PI * control.phase);
    control.trajectory_complete = 0U;
  }

  control.target_foot.x = control.base_foot.x;
  control.target_foot.y = control.base_foot.y -
                          control.profile.lift_mm * shape;
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
          control.profile.target_speed_max);
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
  if (controller_mode() == 0U ||
      motor_index < control.first_motor_index ||
      motor_index >= control.first_motor_index +
                         LEG_TRAJ_MOTOR_COUNT)
    return;
  if (!isfinite(rotor_position) || !isfinite(rotor_velocity) ||
      feedback_timestamp == 0U ||
      (trajectory_mode() != 0U && !update_trajectory_targets(now_ms))) {
    if (control.mode != Motor_LegTrajectory_Stopped)
      Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopController,
                               (int8_t)motor_index, 0.0f);
    return;
  }

  uint8_t local = motor_index - control.first_motor_index;
  Motor_LegTrajectoryChannel *channel = &control.channel[local];
  channel->actual_position = rotor_position;
  channel->raw_velocity = rotor_velocity;
  channel->position_error = channel->target_position - rotor_position;
  ++channel->feedback_count;
  channel->peak_abs_actual_velocity = max_abs(
      channel->peak_abs_actual_velocity, rotor_velocity);
  channel->peak_abs_position_error = max_abs(
      channel->peak_abs_position_error, channel->position_error);

  if ((control.mode == Motor_LegTrajectory_Active ||
       control.mode == Motor_LegTrajectory_Hold) &&
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
      control.profile.position_max) {
    Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopPosition,
                             (int8_t)motor_index,
                             rotor_position - channel->arm_position);
    return;
  }
  /* The motor's reported velocity contains short quantization/filter spikes.
   * A sustained soft limit rejects real overspeed without aborting on a few
   * noisy frames; the independent hard limit still stops a runaway quickly. */
  if (fabsf(rotor_velocity) > control.profile.hard_speed_max) {
    if (channel->hard_overspeed_count < 255U)
      ++channel->hard_overspeed_count;
    if (channel->hard_overspeed_count >=
        MOTOR_LEG_TRAJECTORY_HARD_SPEED_SAMPLES) {
      Motor_LegTrajectory_Stop(Motor_LegTrajectory_StopVelocity,
                               (int8_t)motor_index, rotor_velocity);
      return;
    }
  } else {
    channel->hard_overspeed_count = 0U;
  }
  if (fabsf(rotor_velocity) > control.profile.actual_speed_max) {
    if (channel->overspeed_count < 255U) ++channel->overspeed_count;
    if (channel->overspeed_count >=
        MOTOR_LEG_TRAJECTORY_OVERSPEED_SAMPLES) {
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
      control.profile.target_speed_max);
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
      previous_mode == Motor_LegTrajectory_DryRun) {
    approve_current_plan();
  } else if (previous_mode == Motor_LegTrajectory_Active) {
    Motor_LegTrajectoryApprovedPlan *approved =
        approved_plan_for_level(control.leg_index, control.profile.level);
    if (approved != NULL) approved->valid = 0U;
    control.approved_plan_available = 0U;
    control.dry_run_plan_match = 0U;
    control.dry_run_passed = 0U;
  }
  control.mode = Motor_LegTrajectory_Stopped;
  control.reason = reason;
  control.stop_motor_index = motor_index;
  control.stop_detail = isfinite(detail) ? detail : 0.0f;
  ++control.stop_sequence;
}

uint8_t Motor_LegTrajectory_IsRunning(void)
{
  return controller_mode();
}

int Motor_LegTrajectory_GetAuthorizedTorque(uint8_t motor_index,
                                            float *torque)
{
  if (torque == NULL) return 0;
  *torque = 0.0f;
  if ((control.mode != Motor_LegTrajectory_Active &&
       control.mode != Motor_LegTrajectory_Hold) ||
      motor_index < control.first_motor_index ||
      motor_index >= control.first_motor_index +
                         LEG_TRAJ_MOTOR_COUNT)
    return 0;
  uint8_t local = motor_index - control.first_motor_index;
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
  snapshot->approved_plan_available = control.approved_plan_available;
  snapshot->dry_run_plan_match = control.dry_run_plan_match;
  snapshot->trajectory_complete = control.trajectory_complete;
  snapshot->hold_current_position = control.hold_current_position;
  snapshot->leg_index = control.leg_index;
  snapshot->first_motor_index = control.first_motor_index;
  snapshot->profile = control.profile;
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
    snapshot->plan_arm_position_diff[motor] =
        control.plan_arm_position_diff[motor];
    snapshot->plan_target_delta_diff[motor] =
        control.plan_target_delta_diff[motor];
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
  snapshot->plan_base_foot_diff = control.plan_base_foot_diff;
}
