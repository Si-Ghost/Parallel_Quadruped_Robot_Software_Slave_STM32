#include "Motor_SoftwareControl.h"

#include <math.h>
#include <string.h>

#define SWCTRL_MAX_OFFSET_RAD          1.001f
#define SWCTRL_MAX_KP                  0.50f
#define SWCTRL_MAX_KD                  0.05f
#define SWCTRL_MAX_DURATION_MS         4000U
#define SWCTRL_TARGET_SPEED_RAD_S      0.50f
#define SWCTRL_TORQUE_LIMIT            0.02f
#define SWCTRL_INTEGRAL_LIMIT          0.05f
#define SWCTRL_VELOCITY_FILTER_HZ      100.0f
#define SWCTRL_MIN_DT_S                0.0002f
#define SWCTRL_MAX_DT_S                0.0100f
#define SWCTRL_CASCADE_POSITION_KP     35.9f
#define SWCTRL_CASCADE_POSITION_KI     0.0f
#define SWCTRL_CASCADE_POSITION_KD     1.0f
#define SWCTRL_CASCADE_POSITION_MAX    2.0f
#define SWCTRL_CASCADE_SPEED_KP        0.01f
#define SWCTRL_CASCADE_SPEED_KI        0.0006f
#define SWCTRL_CASCADE_SPEED_KD        0.0015f
#define SWCTRL_CASCADE_TORQUE_MAX      1.50f
#define SWCTRL_CASCADE_INTEGRAL_MAX    1.45f
#define SWCTRL_STATIC_HOLD_DURATION_MS 10000U
#define SWCTRL_STATIC_HOLD_TORQUE_MAX  1.50f
#define SWCTRL_STATIC_HOLD_TARGET_SPEED_MAX 0.50f
#define SWCTRL_STATIC_HOLD_ACTUAL_SPEED_MAX 1.00f
#define SWCTRL_STATIC_HOLD_POSITION_MAX 0.15f
#define SWCTRL_STATIC_HOLD_VELOCITY_FILTER_HZ 20.0f
#define SWCTRL_STATIC_HOLD_INTEGRAL_GATE_RAD 0.0f
/* Two Q15 rotor-position counts.  The dry-run now exercises the same
 * sub-milliradian region that failed to build holding torque in hardware. */
#define SWCTRL_STATIC_HOLD_SIM_OFFSET_RAD (2.0f * 6.28318530718f / 32768.0f)
#define SWCTRL_STATIC_HOLD_BIAS_CAL_MS 2000U
#define SWCTRL_STATIC_HOLD_RAW_OVERSPEED_SAMPLES 5U

static Motor_SoftwareControlSnapshot control;
static uint32_t plan_start_ms;
static uint32_t previous_feedback_timestamp;
static const char *last_reject_reason = "none";
static float cascade_position_error_previous;
static float cascade_speed_error_previous;
static float previous_rotor_position;

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
  control.test = Motor_SoftwareControl_TestNone;
  control.safety_limit = Motor_SoftwareControl_LimitNone;
  control.motor_index = -1;
  control.dry_run = 1U;
  control.torque_limit = SWCTRL_TORQUE_LIMIT;
  control.target_velocity_limit = SWCTRL_TARGET_SPEED_RAD_S;
  control.velocity_filter_hz = SWCTRL_VELOCITY_FILTER_HZ;
  plan_start_ms = 0U;
  previous_feedback_timestamp = 0U;
  cascade_position_error_previous = 0.0f;
  cascade_speed_error_previous = 0.0f;
  previous_rotor_position = 0.0f;
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

int Motor_SoftwareControl_StartCascadeMoveActive(uint8_t motor_index,
                                                 float offset_rad,
                                                 uint32_t duration_ms,
                                                 uint32_t now_ms)
{
  if (control.mode != Motor_SoftwareControl_Armed) last_reject_reason = "not_armed";
  else if (control.motor_index != (int8_t)motor_index) last_reject_reason = "different_armed_motor";
  else if (!isfinite(offset_rad) || offset_rad == 0.0f || fabsf(offset_rad) > SWCTRL_MAX_OFFSET_RAD) last_reject_reason = "offset_limit";
  else if (duration_ms == 0U || duration_ms > SWCTRL_MAX_DURATION_MS) last_reject_reason = "duration_limit";
  else last_reject_reason = "none";

  if (strcmp(last_reject_reason, "none") != 0) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidCommand);
    return 0;
  }
  /* Debug movement uses the same cascade PID as the reference project.
   * Motor_Transport remains the final global output guard. */
  control.mode = Motor_SoftwareControl_CascadeActiveTorque;
  control.dry_run = 0U;
  control.test = Motor_SoftwareControl_TestPositionStep;
  control.safety_limit = Motor_SoftwareControl_LimitNone;
  control.stop_reason = Motor_SoftwareControl_StopNone;
  control.raw_target = control.arm_position + offset_rad;
  control.ramped_target = control.arm_position;
  control.kp = 0.0f;
  control.ki = 0.0f;
  control.kd = 0.0f;
  control.position_loop_kp = SWCTRL_CASCADE_POSITION_KP;
  control.position_loop_ki = SWCTRL_CASCADE_POSITION_KI;
  control.position_loop_kd = SWCTRL_CASCADE_POSITION_KD;
  control.speed_loop_kp = SWCTRL_CASCADE_SPEED_KP;
  control.speed_loop_ki = SWCTRL_CASCADE_SPEED_KI;
  control.speed_loop_kd = SWCTRL_CASCADE_SPEED_KD;
  control.torque_limit = SWCTRL_CASCADE_TORQUE_MAX;
  control.duration_ms = duration_ms;
  control.elapsed_ms = 0U;
  control.i_term = 0.0f;
  plan_start_ms = now_ms;
  /* Arming and starting are separate operator actions. Do not interpret that
   * human delay as one controller sample; the first new feedback after start
   * establishes the dry-run time base. */
  previous_feedback_timestamp = 0U;
  cascade_position_error_previous = 0.0f;
  cascade_speed_error_previous = 0.0f;
  return 1;
}

static void configure_static_hold(Motor_SoftwareControlMode mode,
                                  uint8_t dry_run, uint32_t now_ms)
{
  control.mode = mode;
  control.stop_reason = Motor_SoftwareControl_StopNone;
  control.test = Motor_SoftwareControl_TestStaticHold;
  control.safety_limit = Motor_SoftwareControl_LimitNone;
  control.dry_run = dry_run;
  control.raw_target = control.arm_position;
  control.ramped_target = control.arm_position;
  control.ramp_velocity = 0.0f;
  control.kp = 0.0f;
  control.ki = 0.0f;
  control.kd = 0.0f;
  control.position_loop_kp = SWCTRL_CASCADE_POSITION_KP;
  control.position_loop_ki = SWCTRL_CASCADE_POSITION_KI;
  control.position_loop_kd = SWCTRL_CASCADE_POSITION_KD;
  control.speed_loop_kp = SWCTRL_CASCADE_SPEED_KP;
  control.speed_loop_ki = SWCTRL_CASCADE_SPEED_KI;
  control.speed_loop_kd = SWCTRL_CASCADE_SPEED_KD;
  control.torque_limit = SWCTRL_STATIC_HOLD_TORQUE_MAX;
  control.target_velocity_limit = SWCTRL_STATIC_HOLD_TARGET_SPEED_MAX;
  control.actual_velocity_limit = SWCTRL_STATIC_HOLD_ACTUAL_SPEED_MAX;
  control.position_excursion_limit = SWCTRL_STATIC_HOLD_POSITION_MAX;
  control.velocity_filter_hz = SWCTRL_STATIC_HOLD_VELOCITY_FILTER_HZ;
  control.integral_position_gate = SWCTRL_STATIC_HOLD_INTEGRAL_GATE_RAD;
  control.velocity_bias = 0.0f;
  control.corrected_velocity = 0.0f;
  control.velocity_bias_samples = 0U;
  control.velocity_bias_valid = 0U;
  control.position_velocity = 0.0f;
  control.raw_overspeed_count = 0U;
  control.duration_ms = SWCTRL_STATIC_HOLD_DURATION_MS;
  control.elapsed_ms = 0U;
  control.position_error = 0.0f;
  control.speed_target = 0.0f;
  control.speed_error = 0.0f;
  control.feedback_torque = 0.0f;
  control.p_term = 0.0f;
  control.i_term = 0.0f;
  control.d_term = 0.0f;
  control.calculated_torque = 0.0f;
  control.limited_torque = 0.0f;
  control.torque_limited = 0U;
  control.integral_enabled = 0U;
  control.simulated_position_offset = 0.0f;
  control.simulation_phase = 0U;
  plan_start_ms = now_ms;
  previous_feedback_timestamp = 0U;
  cascade_position_error_previous = 0.0f;
  cascade_speed_error_previous = 0.0f;
  previous_rotor_position = 0.0f;
}

int Motor_SoftwareControl_StartStaticHoldDryRun(uint8_t motor_index,
                                                uint32_t now_ms)
{
  if (control.mode != Motor_SoftwareControl_Armed)
    last_reject_reason = "not_armed";
  else if (control.motor_index != (int8_t)motor_index)
    last_reject_reason = "different_armed_motor";
  else
    last_reject_reason = "none";

  if (strcmp(last_reject_reason, "none") != 0) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidCommand);
    return 0;
  }

  configure_static_hold(Motor_SoftwareControl_StaticHoldDryRun, 1U, now_ms);
  return 1;
}

int Motor_SoftwareControl_StartStaticHoldActive(uint8_t motor_index,
                                                uint32_t now_ms)
{
  if (control.mode != Motor_SoftwareControl_Armed)
    last_reject_reason = "not_armed";
  else if (control.motor_index != (int8_t)motor_index)
    last_reject_reason = "different_armed_motor";
  else
    last_reject_reason = "none";

  if (strcmp(last_reject_reason, "none") != 0) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidCommand);
    return 0;
  }

  configure_static_hold(Motor_SoftwareControl_StaticHoldActiveTorque, 0U,
                        now_ms);
  return 1;
}

static uint8_t static_hold_safety_exceeded(float rotor_position,
                                           float rotor_velocity,
                                           uint8_t position_velocity_valid)
{
  if (control.mode != Motor_SoftwareControl_StaticHoldDryRun &&
      control.mode != Motor_SoftwareControl_StaticHoldActiveTorque) return 0U;
  if (fabsf(rotor_position - control.arm_position) >
      control.position_excursion_limit) {
    control.safety_limit = Motor_SoftwareControl_LimitPositionExcursion;
    return 1U;
  }
  if (position_velocity_valid != 0U &&
      fabsf(control.position_velocity) > control.actual_velocity_limit) {
    control.safety_limit = Motor_SoftwareControl_LimitPositionVelocity;
    return 1U;
  }
  if (fabsf(rotor_velocity) > control.actual_velocity_limit) {
    if (control.raw_overspeed_count < UINT8_MAX)
      ++control.raw_overspeed_count;
  } else {
    control.raw_overspeed_count = 0U;
  }
  if (control.raw_overspeed_count >=
      SWCTRL_STATIC_HOLD_RAW_OVERSPEED_SAMPLES) {
    control.safety_limit = Motor_SoftwareControl_LimitRawVelocity;
    return 1U;
  }
  return 0U;
}

static void update_static_hold_dry_run_simulation(void)
{
  control.simulated_position_offset = 0.0f;
  control.simulation_phase = 0U;
  if (control.mode != Motor_SoftwareControl_StaticHoldDryRun) return;

  if (control.elapsed_ms >= 2000U && control.elapsed_ms < 4000U) {
    control.simulated_position_offset = -SWCTRL_STATIC_HOLD_SIM_OFFSET_RAD;
    control.simulation_phase = 1U;
  } else if (control.elapsed_ms >= 4000U && control.elapsed_ms < 6000U) {
    control.simulation_phase = 2U;
  } else if (control.elapsed_ms >= 6000U && control.elapsed_ms < 8000U) {
    control.simulated_position_offset = SWCTRL_STATIC_HOLD_SIM_OFFSET_RAD;
    control.simulation_phase = 3U;
  } else if (control.elapsed_ms >= 8000U) {
    control.simulation_phase = 4U;
  }
}

static void update_static_hold_velocity_bias(float rotor_velocity)
{
  uint8_t static_hold_mode =
      (control.mode == Motor_SoftwareControl_StaticHoldDryRun ||
       control.mode == Motor_SoftwareControl_StaticHoldActiveTorque) ? 1U : 0U;
  if (!static_hold_mode) {
    control.velocity_bias = 0.0f;
    control.corrected_velocity = control.filtered_velocity;
    control.velocity_bias_samples = 0U;
    control.velocity_bias_valid = 1U;
    return;
  }

  if (control.elapsed_ms < SWCTRL_STATIC_HOLD_BIAS_CAL_MS) {
    ++control.velocity_bias_samples;
    float count = (float)control.velocity_bias_samples;
    control.velocity_bias += (rotor_velocity - control.velocity_bias) / count;
    control.velocity_bias_valid = 0U;
  } else {
    control.velocity_bias_valid = 1U;
  }
  control.corrected_velocity = control.filtered_velocity - control.velocity_bias;
}

void Motor_SoftwareControl_Update(float rotor_position, float rotor_velocity,
                                  float feedback_torque,
                                  uint32_t feedback_timestamp, uint32_t now_ms)
{
  if (control.mode != Motor_SoftwareControl_DryRun &&
      control.mode != Motor_SoftwareControl_ActiveTorque &&
      control.mode != Motor_SoftwareControl_CascadeDryRun &&
      control.mode != Motor_SoftwareControl_CascadeActiveTorque &&
      control.mode != Motor_SoftwareControl_StaticHoldDryRun &&
      control.mode != Motor_SoftwareControl_StaticHoldActiveTorque) return;
  if (!isfinite(rotor_position) || !isfinite(rotor_velocity) ||
      !isfinite(feedback_torque)) {
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopInvalidNumber);
    return;
  }
  if (previous_feedback_timestamp == 0U) {
    previous_feedback_timestamp = feedback_timestamp;
    control.actual_position = rotor_position;
    control.raw_velocity = rotor_velocity;
    control.filtered_velocity = rotor_velocity;
    control.position_velocity = 0.0f;
    previous_rotor_position = rotor_position;
    control.feedback_torque = feedback_torque;
    control.position_error = control.raw_target - rotor_position;
    control.feedback_timestamp = feedback_timestamp;
    control.elapsed_ms = now_ms - plan_start_ms;
    update_static_hold_dry_run_simulation();
    update_static_hold_velocity_bias(rotor_velocity);
    control.position_error = control.raw_target -
                             (rotor_position + control.simulated_position_offset);
    if (static_hold_safety_exceeded(rotor_position, rotor_velocity, 0U))
      Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopSafetyLimit);
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
  float target_step = clampf(remaining, max_step);
  control.ramped_target += target_step;
  control.ramp_velocity = target_step / dt;
  control.actual_position = rotor_position;
  control.raw_velocity = rotor_velocity;
  control.feedback_torque = feedback_torque;
  control.dt_s = dt;
  control.feedback_timestamp = feedback_timestamp;
  control.elapsed_ms = now_ms - plan_start_ms;
  update_static_hold_dry_run_simulation();
  control.position_velocity = (rotor_position - previous_rotor_position) / dt;
  previous_rotor_position = rotor_position;

  if (static_hold_safety_exceeded(rotor_position, rotor_velocity, 1U)) {
    control.position_error = control.raw_target - rotor_position;
    Motor_SoftwareControl_Stop(Motor_SoftwareControl_StopSafetyLimit);
    return;
  }

  const float two_pi = 6.28318530718f;
  float alpha = (two_pi * control.velocity_filter_hz * dt) /
                (1.0f + two_pi * control.velocity_filter_hz * dt);
  control.filtered_velocity += alpha * (rotor_velocity - control.filtered_velocity);
  update_static_hold_velocity_bias(rotor_velocity);
  control.position_error = control.ramped_target -
                           (rotor_position + control.simulated_position_offset);
  uint8_t static_hold_mode =
      (control.mode == Motor_SoftwareControl_StaticHoldDryRun ||
       control.mode == Motor_SoftwareControl_StaticHoldActiveTorque) ? 1U : 0U;
  if (control.mode == Motor_SoftwareControl_CascadeDryRun ||
      control.mode == Motor_SoftwareControl_CascadeActiveTorque ||
      control.mode == Motor_SoftwareControl_StaticHoldDryRun ||
      control.mode == Motor_SoftwareControl_StaticHoldActiveTorque) {
    float previous_position_error = cascade_position_error_previous;
    float position_delta = control.position_error - previous_position_error;
    cascade_position_error_previous = control.position_error;
    float speed_target_limit =
        (control.mode == Motor_SoftwareControl_StaticHoldDryRun ||
         control.mode == Motor_SoftwareControl_StaticHoldActiveTorque)
            ? control.target_velocity_limit : SWCTRL_CASCADE_POSITION_MAX;
    control.speed_target = clampf(control.ramp_velocity +
                                  SWCTRL_CASCADE_POSITION_KP * control.position_error +
                                  SWCTRL_CASCADE_POSITION_KI * 0.0f +
                                  SWCTRL_CASCADE_POSITION_KD * position_delta,
                                  speed_target_limit);
    control.speed_error = control.speed_target - control.corrected_velocity;
    float speed_delta = control.speed_error - cascade_speed_error_previous;
    cascade_speed_error_previous = control.speed_error;
    control.p_term = SWCTRL_CASCADE_SPEED_KP * control.speed_error;
    control.d_term = SWCTRL_CASCADE_SPEED_KD * speed_delta;
  } else {
    control.p_term = control.kp * control.position_error;
    control.d_term = -control.kd * control.filtered_velocity;
  }

  /* The first two seconds are measurement-only.  Future active tests must
   * keep the supported load fully external while the zero-speed bias settles. */
  if (static_hold_mode && control.velocity_bias_valid == 0U) {
    control.p_term = 0.0f;
    control.d_term = 0.0f;
  }

  uint8_t cascade_mode = (control.mode == Motor_SoftwareControl_CascadeDryRun ||
                          control.mode == Motor_SoftwareControl_CascadeActiveTorque ||
                          control.mode == Motor_SoftwareControl_StaticHoldDryRun ||
                          control.mode == Motor_SoftwareControl_StaticHoldActiveTorque);
  float integral_error = cascade_mode
                             ? control.speed_error : control.position_error;
  float integral_gain = cascade_mode
                            ? control.speed_loop_ki : control.ki;
  float integral_step = integral_gain * integral_error;
  control.integral_enabled = 1U;
  /* Match the reference cascade PID semantics after bias calibration: the
   * discrete speed integrator runs every control cycle, including close to
   * the position target.  The previous 1 mrad gate prevented any static load
   * torque from being established. */
  if (static_hold_mode && control.velocity_bias_valid == 0U) {
    integral_step = 0.0f;
    control.integral_enabled = 0U;
  }
  if (!cascade_mode) integral_step *= dt;
  float integral_limit = cascade_mode
                             ? SWCTRL_CASCADE_INTEGRAL_MAX : SWCTRL_INTEGRAL_LIMIT;
  float i_candidate = clampf(control.i_term + integral_step, integral_limit);
  float unsaturated = control.p_term + i_candidate + control.d_term;
  float limited = clampf(unsaturated, control.torque_limit);
  if (limited == unsaturated || (integral_error * unsaturated) < 0.0f)
    control.i_term = i_candidate;
  control.calculated_torque = control.p_term + control.i_term + control.d_term;
  control.limited_torque = clampf(control.calculated_torque,
                                  control.torque_limit);
  control.torque_limited =
      (control.limited_torque != control.calculated_torque) ? 1U : 0U;
}

void Motor_SoftwareControl_GetSnapshot(Motor_SoftwareControlSnapshot *snapshot)
{
  if (snapshot != NULL) *snapshot = control;
}

const char *Motor_SoftwareControl_GetLastRejectReason(void)
{
  return last_reject_reason;
}

int Motor_SoftwareControl_GetAuthorizedTorque(uint8_t motor_index, float *torque)
{
  if (torque == NULL) return 0;
  *torque = 0.0f;
  if ((control.mode != Motor_SoftwareControl_CascadeActiveTorque &&
       control.mode != Motor_SoftwareControl_StaticHoldActiveTorque) ||
      control.motor_index != (int8_t)motor_index || control.dry_run != 0U)
    return 0;
  if (!isfinite(control.limited_torque) ||
      !isfinite(control.torque_limit) || control.torque_limit <= 0.0f ||
      fabsf(control.limited_torque) > control.torque_limit)
    return 0;
  *torque = control.limited_torque;
  return 1;
}
