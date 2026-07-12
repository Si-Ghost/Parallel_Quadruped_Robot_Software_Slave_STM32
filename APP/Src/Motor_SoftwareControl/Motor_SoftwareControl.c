#include "Motor_SoftwareControl.h"

#include <math.h>
#include <string.h>

#define SWCTRL_MAX_OFFSET_RAD          0.30f
#define SWCTRL_MAX_KP                  0.50f
#define SWCTRL_MAX_KD                  0.05f
#define SWCTRL_MAX_DURATION_MS         1500U
#define SWCTRL_TARGET_SPEED_RAD_S      0.20f
#define SWCTRL_TORQUE_LIMIT            0.20f
#define SWCTRL_INTEGRAL_LIMIT          0.05f
#define SWCTRL_VELOCITY_FILTER_HZ      30.0f
#define SWCTRL_MIN_DT_S                0.0002f
#define SWCTRL_MAX_DT_S                0.0100f

static Motor_SoftwareControlSnapshot control;
static uint32_t plan_start_ms;
static uint32_t previous_feedback_timestamp;
static const char *last_reject_reason = "none";

static float clampf(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void Motor_SoftwareControl_Init(void)
{
  memset(&control, 0, sizeof(control));
  control.mode = Motor_SoftwareControl_Disabled;
  control.motor_index = -1;
  control.dry_run = 1U;
  control.torque_limit = SWCTRL_TORQUE_LIMIT;
  control.target_velocity_limit = SWCTRL_TARGET_SPEED_RAD_S;
  plan_start_ms = 0U;
  previous_feedback_timestamp = 0U;
  last_reject_reason = "none";
}

void Motor_SoftwareControl_Stop(Motor_SoftwareControlStopReason reason)
{
  control.mode = Motor_SoftwareControl_Stopped;
  control.stop_reason = reason;
  control.i_term = 0.0f;
  control.calculated_torque = 0.0f;
  control.limited_torque = 0.0f;
  control.torque_limited = 0U;
  previous_feedback_timestamp = 0U;
}

int Motor_SoftwareControl_Arm(uint8_t motor_index, float rotor_position,
                              uint32_t feedback_timestamp)
{
  if (motor_index >= 8U || !isfinite(rotor_position) || feedback_timestamp == 0U) {
    last_reject_reason = "invalid_arm_feedback";
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidCommand);
    return 0;
  }
  Motor_SoftwareControl_Init();
  control.mode = Motor_SoftwareControl_Armed;
  control.motor_index = (int8_t)motor_index;
  control.arm_position = rotor_position;
  control.raw_target = rotor_position;
  control.ramped_target = rotor_position;
  control.actual_position = rotor_position;
  control.feedback_timestamp = feedback_timestamp;
  previous_feedback_timestamp = feedback_timestamp;
  return 1;
}

int Motor_SoftwareControl_StartDryRun(uint8_t motor_index, float offset_rad,
                                      float kp, float kd, uint32_t duration_ms,
                                      uint32_t now_ms)
{
  if (control.mode != Motor_SoftwareControl_Armed) last_reject_reason = "not_armed";
  else if (control.motor_index != (int8_t)motor_index) last_reject_reason = "different_armed_motor";
  else if (!isfinite(offset_rad) || offset_rad == 0.0f || fabsf(offset_rad) > SWCTRL_MAX_OFFSET_RAD) last_reject_reason = "offset_limit";
  else if (!isfinite(kp) || kp < 0.0f || kp > SWCTRL_MAX_KP) last_reject_reason = "kp_limit";
  else if (!isfinite(kd) || kd < 0.0f || kd > SWCTRL_MAX_KD) last_reject_reason = "kd_limit";
  else if (duration_ms == 0U || duration_ms > SWCTRL_MAX_DURATION_MS) last_reject_reason = "duration_limit";
  else last_reject_reason = "none";

  if (strcmp(last_reject_reason, "none") != 0) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidCommand);
    return 0;
  }
  control.mode = Motor_SoftwareControl_DryRun;
  control.stop_reason = Motor_SoftwareControl_StopNone;
  control.raw_target = control.arm_position + offset_rad;
  control.ramped_target = control.arm_position;
  control.kp = kp;
  control.ki = 0.0f;
  control.kd = kd;
  control.duration_ms = duration_ms;
  control.elapsed_ms = 0U;
  control.i_term = 0.0f;
  plan_start_ms = now_ms;
  /* Arming and starting are separate operator actions. Do not interpret that
   * human delay as one controller sample; the first new feedback after start
   * establishes the dry-run time base. */
  previous_feedback_timestamp = 0U;
  return 1;
}

void Motor_SoftwareControl_Update(float rotor_position, float rotor_velocity,
                                  uint32_t feedback_timestamp, uint32_t now_ms)
{
  if (control.mode != Motor_SoftwareControl_DryRun) return;
  if (!isfinite(rotor_position) || !isfinite(rotor_velocity)) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidNumber);
    return;
  }
  if (previous_feedback_timestamp == 0U) {
    previous_feedback_timestamp = feedback_timestamp;
    control.actual_position = rotor_position;
    control.raw_velocity = rotor_velocity;
    control.feedback_timestamp = feedback_timestamp;
    control.elapsed_ms = now_ms - plan_start_ms;
    return;
  }
  if (feedback_timestamp == previous_feedback_timestamp) return;

  float dt = (float)(feedback_timestamp - previous_feedback_timestamp) * 0.001f;
  previous_feedback_timestamp = feedback_timestamp;
  if (!isfinite(dt) || dt < SWCTRL_MIN_DT_S || dt > SWCTRL_MAX_DT_S) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidDt);
    return;
  }

  float remaining = control.raw_target - control.ramped_target;
  float max_step = control.target_velocity_limit * dt;
  control.ramped_target += clampf(remaining, max_step);
  control.actual_position = rotor_position;
  control.raw_velocity = rotor_velocity;
  control.dt_s = dt;
  control.feedback_timestamp = feedback_timestamp;
  control.elapsed_ms = now_ms - plan_start_ms;

  const float two_pi = 6.28318530718f;
  float alpha = (two_pi * SWCTRL_VELOCITY_FILTER_HZ * dt) /
                (1.0f + two_pi * SWCTRL_VELOCITY_FILTER_HZ * dt);
  control.filtered_velocity += alpha * (rotor_velocity - control.filtered_velocity);
  control.position_error = control.ramped_target - rotor_position;
  control.p_term = control.kp * control.position_error;
  control.d_term = -control.kd * control.filtered_velocity;

  float i_candidate = clampf(control.i_term + control.ki * control.position_error * dt,
                             SWCTRL_INTEGRAL_LIMIT);
  float unsaturated = control.p_term + i_candidate + control.d_term;
  float limited = clampf(unsaturated, control.torque_limit);
  if (limited == unsaturated || (control.position_error * unsaturated) < 0.0f)
    control.i_term = i_candidate;
  control.calculated_torque = control.p_term + control.i_term + control.d_term;
  control.limited_torque = clampf(control.calculated_torque, control.torque_limit);
  control.torque_limited = (control.limited_torque != control.calculated_torque) ? 1U : 0U;
}

void Motor_SoftwareControl_GetSnapshot(Motor_SoftwareControlSnapshot *snapshot)
{
  if (snapshot != NULL) *snapshot = control;
}

const char *Motor_SoftwareControl_GetLastRejectReason(void)
{
  return last_reject_reason;
}
