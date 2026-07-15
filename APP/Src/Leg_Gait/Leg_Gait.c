#include "Leg_Gait.h"
#include "Leg_Kinematics.h"
#include "Leg_Control.h"
#include "communication.h"
#include "GO-M8010-6.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern Leg_HandlerTypeDef* Legs[4];
extern RC_DataTypeDef rc_data;

#define LEG_TARGET_RAMP_MS          1200U
#define LEG_ALL_MICRO_DY_MM         3.0f
#define LEG_PREP_POSE_DY_MM         5.0f
#define LEG_TRACE_POINT_COUNT       4U
#define LEG_FOOT_NUDGE_MAX_MM       15.0f
#define LEG_FOOT_NUDGE_ROTOR_RAD    0.65f
#define LEG_WEB_STOP_ERROR_RAD      0.08f
#define LEG_ALL_MICRO_STOP_ERROR_RAD 0.05f
#define LEG_SINE_AMP_MAX_MM        10.0f
#define LEG_SINE_AMP_MIN_MM         1.0f
#define LEG_SINE_FREQ_MAX_HZ        2.0f
#define LEG_SINE_FREQ_MIN_HZ        0.1f
#define LEG_SINE_LOG_PERIOD_MS      200U
#define LEG_TROT_RETURN_MS          600U
#define LEG_TROT_HOLD_SETTLE_MS     200U
#define LEG_TROT_LOG_PERIOD_MS      500U
#define LEG_TROT_BASE_X_MM            0.0f
#define LEG_TROT_ZERO_Y_MM          225.0f
#define LEG_REMOTE_CHANNEL_DEADZONE 363
#define LEG_TROT_PROFILE_COUNT        3U
#define LEG_TROT_INTERNAL_PD_UI_S2    1U
#define LEG_TROT_INTERNAL_PD_KP       6.0f
#define LEG_TROT_INTERNAL_PD_KW       0.35f

typedef struct
{
  uint8_t active;
  uint8_t leg;
  uint8_t step;
  Leg_PointTypeDef base_foot;
} Leg_DebugTraceTypeDef;

typedef struct
{
  uint8_t active;
  uint8_t phase;
  Leg_PointTypeDef base_foot[4];
} Leg_AllMicroTypeDef;

typedef struct
{
  uint8_t active;
  Leg_PointTypeDef base_foot[4];
} Leg_PrepPoseTypeDef;

typedef struct
{
  uint8_t active;
  uint8_t leg;
  Leg_PointTypeDef base_foot;
  float amplitude_mm;
  float freq_hz;
  uint32_t start_tick;
  uint32_t last_log_tick;
} Leg_SineTypeDef;

typedef struct
{
  uint8_t level;
  uint8_t ui_s2;
  uint8_t ref_s2;
  float lift_height_mm;
  float start_point_mm;
  float base_y_mm;
  uint32_t half_cycle_ms;
  uint32_t cycle_ms;
  uint32_t entry_ms;
  float swing_fraction;
  float max_zero_excursion_rad;
} Leg_TrotProfileTypeDef;

typedef struct
{
  uint8_t active;
  uint8_t stage;
  uint8_t profile_index;
  int8_t direction;
  uint8_t stop_requested;
  uint8_t one_cycle;
  uint32_t stage_start_tick;
  uint32_t stop_after_tick;
  uint32_t last_log_tick;
  float zero_targets[8];
  float return_start_targets[8];
  Leg_PointTypeDef target_foot[4];
  char leg_state[4];
} Leg_TrotTypeDef;

typedef struct
{
  uint8_t pending;
  uint8_t line_index;
  uint8_t level;
  uint8_t ui_s2;
  uint8_t stage;
  int8_t direction;
  uint8_t stop_requested;
  uint32_t timestamp;
  uint32_t cycle_ms;
  uint32_t entry_ms;
  float step_mm;
  float lift_mm;
  float phase;
  float blend;
  Leg_PointTypeDef foot[4];
  char leg_state[4];
  Motor_GroupSnapshot group;
  Motor_StateSnapshotTypeDef motor[8];
} Leg_TrotTelemetryTypeDef;

static Leg_DebugTraceTypeDef debug_trace = {0};
static Leg_AllMicroTypeDef all_micro = {0};
static Leg_PrepPoseTypeDef prep_pose = {0};
static Leg_SineTypeDef sine_test = {0};
static Leg_TrotTypeDef trot = {0};
static Leg_TrotTelemetryTypeDef trot_telemetry = {0};
static uint8_t trot_result_pending = 0U;
static char trot_result_line[256];
static uint8_t remote_armed = 0U;
static uint8_t remote_last_s1 = 0U;
static uint8_t remote_last_s2 = 0U;
static uint32_t remote_last_reject_log_tick = 0U;

/* UI gears are ordered by operator speed, while Software_Ref names its
 * profiles S2=2 slow, S2=3 medium and S2=1 fast.  Keep that mapping explicit
 * so the reference timing is preserved without exposing its non-monotonic
 * numbering to the operator.  This first internal-PD migration authorizes
 * only the current lowest UI profile (S2=1); the other definitions remain
 * visible for later staged expansion but configure_trot_profile rejects them. */
static const Leg_TrotProfileTypeDef trot_profiles[LEG_TROT_PROFILE_COUNT] = {
    {5U, 1U, 2U, 30.0f, 40.0f, 220.0f, 300U, 600U, 600U,
     0.50f, 4.00f},
    {6U, 2U, 3U, 50.0f, 75.0f, 215.0f, 220U, 440U, 440U,
     0.50f, 4.30f},
    {7U, 3U, 1U, 40.0f, 75.0f, 210.0f, 175U, 350U, 350U,
     0.50f, 4.00f},
};

static uint8_t trot_profile_index_from_s2(uint8_t s2)
{
  return s2 >= 1U && s2 <= LEG_TROT_PROFILE_COUNT
             ? (uint8_t)(s2 - 1U)
             : 0U;
}

static int configure_trot_profile(const Leg_TrotProfileTypeDef *profile)
{
  if (profile == NULL || profile->ui_s2 != LEG_TROT_INTERNAL_PD_UI_S2)
    return 0;
  Motor_GroupGaitLimits limits = {
      .max_zero_excursion_rad = profile->max_zero_excursion_rad,
      .command_backend = Motor_Group_CommandInternalPd,
      .internal_pd_kp = LEG_TROT_INTERNAL_PD_KP,
      .internal_pd_kw = LEG_TROT_INTERNAL_PD_KW,
  };
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  int configured = Motor_GroupControl_ConfigureGait(&limits);
  if (primask == 0U) __enable_irq();
  return configured;
}

static int trot_rotor_targets_to_foot(uint8_t leg,
                                      const float rotor_targets[8],
                                      Leg_PointTypeDef *foot)
{
  if (leg >= 4U || rotor_targets == NULL || foot == NULL) return 0;
  uint8_t first = leg * 2U;
  Leg_JointAnglesTypeDef angles = {
      .theta1 = Leg_Control_NormalizedMotorDir(
                    Legs[leg]->motor_direction[0]) *
                (rotor_targets[first] - Legs[leg]->rotor_zero_offset[0]) /
                LEG_REDUCTION_RATIO,
      .theta2 = Leg_Control_NormalizedMotorDir(
                    Legs[leg]->motor_direction[1]) *
                (rotor_targets[first + 1U] -
                 Legs[leg]->rotor_zero_offset[1]) /
                LEG_REDUCTION_RATIO,
  };
  return Leg_Kinematics_Forward(&angles, foot);
}

static int set_trot_targets(const float rotor_targets[8])
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  int target_ok = Motor_GroupControl_SetGaitTargets(rotor_targets);
  if (primask == 0U) __enable_irq();
  return target_ok;
}

static int begin_trot_return(uint32_t now, const char *log_line)
{
  Motor_GroupSnapshot group;
  Leg_Control_GetGroupSnapshot(&group);
  if (group.mode != Motor_Group_Active ||
      group.profile != Motor_Group_ProfileGait)
    return 0;
  for (uint8_t i = 0U; i < 8U; ++i)
    trot.return_start_targets[i] = group.target_position[i];
  trot.stage = 3U;
  trot.stage_start_tick = now;
  trot.stop_after_tick = 0U;
  for (uint8_t leg = 0U; leg < 4U; ++leg)
    trot.leg_state[leg] = 'R';
  if (log_line != NULL) Communication_SendString(log_line);
  return 1;
}

static void capture_trot_telemetry(uint32_t now,
                                   const Leg_TrotProfileTypeDef *profile,
                                   float phase,
                                   float blend)
{
  if (profile == NULL || trot_telemetry.pending != 0U) return;
  memset(&trot_telemetry, 0, sizeof(trot_telemetry));
  trot_telemetry.pending = 1U;
  trot_telemetry.level = profile->level;
  trot_telemetry.ui_s2 = profile->ui_s2;
  trot_telemetry.stage = trot.stage;
  trot_telemetry.direction = trot.direction;
  trot_telemetry.stop_requested = trot.stop_requested;
  trot_telemetry.timestamp = now;
  trot_telemetry.cycle_ms = profile->cycle_ms;
  trot_telemetry.entry_ms = profile->entry_ms;
  trot_telemetry.step_mm = profile->start_point_mm * 2.0f;
  trot_telemetry.lift_mm = profile->lift_height_mm;
  trot_telemetry.phase = phase;
  trot_telemetry.blend = blend;
  for (uint8_t leg = 0U; leg < 4U; ++leg) {
    trot_telemetry.foot[leg] = trot.target_foot[leg];
    trot_telemetry.leg_state[leg] = trot.leg_state[leg];
  }
  Leg_Control_GetGroupSnapshot(&trot_telemetry.group);
  for (uint8_t i = 0U; i < 8U; ++i)
    (void)Leg_Control_GetMotorStateSnapshot(i, &trot_telemetry.motor[i]);
}

static void service_trot_telemetry(void)
{
  if (trot_telemetry.pending == 0U) return;
  char line[256];
  int len = -1;
  if (trot_telemetry.line_index == 0U) {
    len = snprintf(
        line, sizeof(line),
        "LEG_TROT_STATE backend=internal_pd level=%u s2=%u stage=%u dir=%d "
        "phase_ppm=%ld blend_ppm=%ld stop=%u step_mm=%ld lift_mm=%ld "
        "cycle_ms=%lu entry_ms=%lu return_ms=%u\r\n",
        (unsigned int)trot_telemetry.level,
        (unsigned int)trot_telemetry.ui_s2,
        (unsigned int)trot_telemetry.stage,
        (int)trot_telemetry.direction,
        (long)(trot_telemetry.phase * 1000000.0f),
        (long)(trot_telemetry.blend * 1000000.0f),
        (unsigned int)trot_telemetry.stop_requested,
        (long)trot_telemetry.step_mm, (long)trot_telemetry.lift_mm,
        (unsigned long)trot_telemetry.cycle_ms,
        (unsigned long)trot_telemetry.entry_ms,
        (unsigned int)LEG_TROT_RETURN_MS);
  } else if (trot_telemetry.line_index == 1U) {
    len = snprintf(
        line, sizeof(line),
        "LEG_TROT_FEET backend=internal_pd units=um "
        "legend=S:swing,G:stance,R:return,H:hold "
        "L0=%c(%ld,%ld) L1=%c(%ld,%ld) L2=%c(%ld,%ld) L3=%c(%ld,%ld)\r\n",
        trot_telemetry.leg_state[0],
        (long)(trot_telemetry.foot[0].x * 1000.0f),
        (long)(trot_telemetry.foot[0].y * 1000.0f),
        trot_telemetry.leg_state[1],
        (long)(trot_telemetry.foot[1].x * 1000.0f),
        (long)(trot_telemetry.foot[1].y * 1000.0f),
        trot_telemetry.leg_state[2],
        (long)(trot_telemetry.foot[2].x * 1000.0f),
        (long)(trot_telemetry.foot[2].y * 1000.0f),
        trot_telemetry.leg_state[3],
        (long)(trot_telemetry.foot[3].x * 1000.0f),
        (long)(trot_telemetry.foot[3].y * 1000.0f));
  } else {
    uint8_t i = (uint8_t)(trot_telemetry.line_index - 2U);
    Motor_StateSnapshotTypeDef *motor = &trot_telemetry.motor[i];
    float target = trot_telemetry.group.target_position[i];
    uint32_t age = motor->timestamp == 0U
                       ? 9999U
                       : trot_telemetry.timestamp >= motor->timestamp
                             ? trot_telemetry.timestamp - motor->timestamp
                             : 0U;
    if (age > 9999U) age = 9999U;
    len = snprintf(
        line, sizeof(line),
        "LEG_TROT_MOTOR backend=internal_pd i=%u pos_urad=%ld "
        "actual_urad=%ld err_urad=%ld kp_milli=%ld kw_milli=%ld "
        "t_mNm=%ld temp_c=%d online=%u age_ms=%lu fault=%u\r\n",
        (unsigned int)i, (long)(target * 1000000.0f),
        (long)(motor->rotor_position * 1000000.0f),
        (long)((target - motor->rotor_position) * 1000000.0f),
        (long)(LEG_TROT_INTERNAL_PD_KP * 1000.0f),
        (long)(LEG_TROT_INTERNAL_PD_KW * 1000.0f),
        (long)(motor->raw_torque * 1000.0f),
        (int)motor->temperature_c, (unsigned int)motor->online,
        (unsigned long)age, (unsigned int)motor->motor_error);
  }

  if (len <= 0 || len >= (int)sizeof(line)) {
    trot_telemetry.pending = 0U;
    return;
  }
  if (!Communication_TrySendString(line)) return;
  ++trot_telemetry.line_index;
  if (trot_telemetry.line_index >= 10U)
    trot_telemetry.pending = 0U;
}

static const Leg_PointTypeDef trace_offsets[LEG_TRACE_POINT_COUNT] = {
    {0.0f, 5.0f},
    {5.0f, 5.0f},
    {-5.0f, 5.0f},
    {0.0f, 0.0f},
};

static void log_trace_point(const char *tag, uint8_t leg, uint8_t step,
                            const Leg_PointTypeDef *target, const Leg_PointTypeDef *current)
{
  if (tag == NULL || target == NULL || current == NULL)
    return;

  char buf[160];
  int len = snprintf(buf, sizeof(buf), "LEG_TRACE %s leg=%u step=%u tgt=", tag, leg, step);
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, target->x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, target->y);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " cur=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, current->x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, current->y);
  if (len > 0)
  {
    int written = snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");
    if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
      return;
    len += written;
  }

  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void log_trace_simple(const char *tag, uint8_t leg, uint8_t step, Motor_TargetResultTypeDef result)
{
  char buf[80];
  int len = snprintf(buf, sizeof(buf), "LEG_TRACE %s leg=%u step=%u res=%u\r\n",
                     tag, leg, step, result);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void log_all_micro_simple(const char *tag, uint8_t phase, Motor_TargetResultTypeDef result)
{
  char buf[80];
  int len = snprintf(buf, sizeof(buf), "LEG_ALL_MICRO %s phase=%u res=%u\r\n",
                     tag, phase, result);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void log_all_micro_feet(const char *tag, uint8_t phase, Motor_TargetResultTypeDef result)
{
  Leg_PointTypeDef foot[4] = {0};
  uint8_t ok[4] = {0};

  for (uint8_t leg = 0; leg < 4; leg++)
    ok[leg] = Leg_Control_GetCurrentFoot(leg, &foot[leg]) ? 1U : 0U;

  char buf[192];
  int len = snprintf(buf, sizeof(buf), "LEG_ALL_MICRO %s phase=%u res=%u ok=%u%u%u%u y=",
                     tag, phase, result, ok[0], ok[1], ok[2], ok[3]);
  for (uint8_t lg = 0; lg < 4 && len > 0; lg++)
  {
    if (lg > 0)
      len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
    len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, foot[lg].y);
  }
  if (len > 0)
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void log_prep_pose_feet(const char *tag, Motor_TargetResultTypeDef result)
{
  Leg_PointTypeDef foot[4] = {0};
  uint8_t ok[4] = {0};

  for (uint8_t leg = 0; leg < 4; leg++)
    ok[leg] = Leg_Control_GetCurrentFoot(leg, &foot[leg]) ? 1U : 0U;

  char buf[192];
  int len = snprintf(buf, sizeof(buf), "LEG_PREP_POSE %s res=%u ok=%u%u%u%u y=",
                     tag, result, ok[0], ok[1], ok[2], ok[3]);
  for (uint8_t lg = 0; lg < 4 && len > 0; lg++)
  {
    if (lg > 0)
      len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
    len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, foot[lg].y);
  }
  if (len > 0)
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");

  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

int Leg_Gait_AnyActive(void)
{
  return (debug_trace.active || all_micro.active || prep_pose.active ||
          sine_test.active || trot.active) ? 1 : 0;
}

void Leg_Gait_StopAll(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (trot.active) {
    if (Motor_GroupControl_ReturnGaitToZero())
      (void)Motor_GroupControl_FinishGaitHold();
  }
  debug_trace.active = 0U;
  all_micro.active = 0U;
  prep_pose.active = 0U;
  sine_test.active = 0U;
  trot.active = 0U;
  trot_telemetry.pending = 0U;
  if (primask == 0U) __enable_irq();
}

static int start_debug_trace_step(void)
{
  if (!debug_trace.active || debug_trace.leg >= 4 || debug_trace.step >= LEG_TRACE_POINT_COUNT)
    return 0;

  Leg_PointTypeDef current;
  if (!Leg_Control_GetCurrentFoot(debug_trace.leg, &current))
  {
    log_trace_simple("abort_fk", debug_trace.leg, debug_trace.step, Motor_Target_Stall);
    debug_trace.active = 0;
    return 0;
  }

  Leg_PointTypeDef target = {
      .x = debug_trace.base_foot.x + trace_offsets[debug_trace.step].x,
      .y = debug_trace.base_foot.y + trace_offsets[debug_trace.step].y,
  };
  log_trace_point("point", debug_trace.leg, debug_trace.step, &target, &current);

  if (!Leg_Control_SetDebugFootOffset(debug_trace.leg,
                                      target.x - current.x,
                                      target.y - current.y))
  {
    log_trace_simple("abort_start", debug_trace.leg, debug_trace.step, Motor_Target_Stopped);
    debug_trace.active = 0;
    return 0;
  }

  return 1;
}

void Leg_Gait_ServiceDebugTrace(void)
{
  if (!debug_trace.active || debug_trace.leg >= 4)
    return;

  Leg_HandlerTypeDef *hleg = Legs[debug_trace.leg];
  Motor_RuntimeStateTypeDef *m0 = &hleg->motor_state[0];
  Motor_RuntimeStateTypeDef *m1 = &hleg->motor_state[1];
  if (m0->target_active == Motor_Target_Active || m1->target_active == Motor_Target_Active)
    return;

  if (m0->target_result == Motor_Target_Stall || m1->target_result == Motor_Target_Stall ||
      m0->target_result == Motor_Target_Timeout || m1->target_result == Motor_Target_Timeout ||
      m0->target_result == Motor_Target_Stopped || m1->target_result == Motor_Target_Stopped)
  {
    log_trace_simple("abort", debug_trace.leg, debug_trace.step,
                     m0->target_result > m1->target_result ? m0->target_result : m1->target_result);
    debug_trace.active = 0;
    return;
  }

  if (m0->target_result != Motor_Target_Done || m1->target_result != Motor_Target_Done)
    return;

  Leg_PointTypeDef current;
  if (Leg_Control_GetCurrentFoot(debug_trace.leg, &current))
  {
    Leg_PointTypeDef target = {
        .x = debug_trace.base_foot.x + trace_offsets[debug_trace.step].x,
        .y = debug_trace.base_foot.y + trace_offsets[debug_trace.step].y,
    };
    log_trace_point("done", debug_trace.leg, debug_trace.step, &target, &current);
  }

  debug_trace.step++;
  if (debug_trace.step >= LEG_TRACE_POINT_COUNT)
  {
    log_trace_simple("complete", debug_trace.leg, debug_trace.step, Motor_Target_Done);
    debug_trace.active = 0;
    return;
  }

  start_debug_trace_step();
}

static int start_all_micro_phase(uint8_t phase)
{
  float offsets[4][2] = {0};

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    Leg_PointTypeDef target = all_micro.base_foot[leg];
    if (phase == 0U)
      target.y += LEG_ALL_MICRO_DY_MM;

    if (!Leg_Control_ComputeFootTargetOffsets(leg, &target, offsets[leg]))
    {
      log_all_micro_simple("reject_plan", phase, Motor_Target_Stopped);
      return 0;
    }
  }

  all_micro.phase = phase;
  log_all_micro_feet(phase == 0U ? "start" : "return", phase, Motor_Target_Running);

  for (uint8_t lg = 0; lg < 4; lg++)
  {
    for (uint8_t motor = 0; motor < 2; motor++)
    {
      uint8_t idx = Leg_Control_MotorIndex(lg, motor);
      Leg_Control_StartOffsetWithStopError(idx, offsets[lg][motor], LEG_ALL_MICRO_STOP_ERROR_RAD);
      if (!Leg_Control_ApplyDebugTarget(idx, 0))
      {
        for (uint8_t stop_idx = 0; stop_idx < 8; stop_idx++)
          Leg_Control_StopDebugTarget(stop_idx, Motor_Target_Stopped);
        log_all_micro_simple("reject_apply", phase, Motor_Target_Stopped);
        return 0;
      }
    }
  }

  return 1;
}

void Leg_Gait_ServiceAllMicro(void)
{
  if (!all_micro.active)
    return;

  uint8_t all_done = 1U;
  Motor_TargetResultTypeDef bad_result = Motor_Target_Idle;
  for (uint8_t idx = 0; idx < 8; idx++)
  {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state == NULL)
      continue;

    if (state->target_active == Motor_Target_Active)
      return;

    if (state->target_result == Motor_Target_Stall ||
        state->target_result == Motor_Target_Timeout ||
        state->target_result == Motor_Target_Stopped)
    {
      bad_result = state->target_result;
      break;
    }

    if (state->target_result != Motor_Target_Done)
      all_done = 0U;
  }

  if (bad_result != Motor_Target_Idle)
  {
    log_all_micro_feet("abort", all_micro.phase, bad_result);
    all_micro.active = 0U;
    return;
  }

  if (!all_done)
    return;

  log_all_micro_feet("done", all_micro.phase, Motor_Target_Done);

  if (all_micro.phase == 0U)
  {
    if (!start_all_micro_phase(1U))
    {
      all_micro.active = 0U;
      log_all_micro_simple("abort_return", 1U, Motor_Target_Stopped);
    }
    return;
  }

  all_micro.active = 0U;
  log_all_micro_feet("complete", all_micro.phase, Motor_Target_Done);
}

static int start_prep_pose_targets(void)
{
  float offsets[4][2] = {0};

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    Leg_PointTypeDef target = prep_pose.base_foot[leg];
    target.y += LEG_PREP_POSE_DY_MM;

    if (!Leg_Control_ComputeFootTargetOffsets(leg, &target, offsets[leg]))
    {
      log_prep_pose_feet("reject_plan", Motor_Target_Stopped);
      return 0;
    }
  }

  log_prep_pose_feet("start", Motor_Target_Running);

  for (uint8_t lg = 0; lg < 4; lg++)
  {
    for (uint8_t motor = 0; motor < 2; motor++)
    {
      uint8_t idx = Leg_Control_MotorIndex(lg, motor);
      Leg_Control_StartOffsetWithStopError(idx, offsets[lg][motor], LEG_WEB_STOP_ERROR_RAD);
      if (!Leg_Control_ApplyDebugTarget(idx, 0))
      {
        for (uint8_t stop_idx = 0; stop_idx < 8; stop_idx++)
          Leg_Control_StopDebugTarget(stop_idx, Motor_Target_Stopped);
        log_prep_pose_feet("reject_apply", Motor_Target_Stopped);
        return 0;
      }
    }
  }

  return 1;
}

void Leg_Gait_ServicePrepPose(void)
{
  if (!prep_pose.active)
    return;

  uint8_t all_done = 1U;
  Motor_TargetResultTypeDef bad_result = Motor_Target_Idle;
  for (uint8_t idx = 0; idx < 8; idx++)
  {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state == NULL)
      continue;

    if (state->target_active == Motor_Target_Active)
      return;

    if (state->target_result == Motor_Target_Stall ||
        state->target_result == Motor_Target_Timeout ||
        state->target_result == Motor_Target_Stopped)
    {
      bad_result = state->target_result;
      break;
    }

    if (state->target_result != Motor_Target_Done)
      all_done = 0U;
  }

  if (bad_result != Motor_Target_Idle)
  {
    log_prep_pose_feet("abort", bad_result);
    prep_pose.active = 0U;
    return;
  }

  if (!all_done)
    return;

  log_prep_pose_feet("done", Motor_Target_Done);
  prep_pose.active = 0U;
  (void)Leg_Control_HoldCurrentPosition();
  log_prep_pose_feet("complete", Motor_Target_Done);
}

void Leg_Gait_ServiceSine(void)
{
  if (!sine_test.active || sine_test.leg >= 4)
    return;

  uint8_t leg = sine_test.leg;
  for (uint8_t motor = 0; motor < 2; motor++)
  {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    if (state->online != Motor_Online || state->angle_valid != Motor_Angle_Valid)
    {
      Communication_SendString("LEG_SINE abort offline\r\n");
      sine_test.active = 0U;
      return;
    }
  }

  uint32_t now = HAL_GetTick();
  float t = (float)(now - sine_test.start_tick) / 1000.0f;
  float dy = sine_test.amplitude_mm * sinf(LEG_TWO_PI * sine_test.freq_hz * t);

  Leg_PointTypeDef target_foot = {
      .x = sine_test.base_foot.x,
      .y = sine_test.base_foot.y + dy,
  };
  Leg_JointAnglesTypeDef target_angles;
  if (!Leg_Kinematics_Inverse(&target_foot, &target_angles))
  {
    Communication_SendString("LEG_SINE abort ik_fail\r\n");
    sine_test.active = 0U;
    return;
  }

  float rotor_targets[2];
  if (!Leg_Control_JointToRotorTargets(leg, &target_angles, rotor_targets))
  {
    Communication_SendString("LEG_SINE abort rotor_target_fail\r\n");
    sine_test.active = 0U;
    return;
  }

  for (uint8_t motor = 0; motor < 2; motor++)
  {
    MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
    cmd->mode = 1;
    cmd->T = 0.0f;
    cmd->W = 0.0f;
    cmd->Pos = rotor_targets[motor];
    cmd->K_P = LEG_WEB_KP;
    cmd->K_W = LEG_WEB_KW;
    modify_data(cmd);
  }

  if ((now - sine_test.last_log_tick) >= LEG_SINE_LOG_PERIOD_MS)
  {
    sine_test.last_log_tick = now;

    for (uint8_t motor = 0; motor < 2; motor++)
    {
      Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
      float q_des = rotor_targets[motor];
      float q_act = state->angle;
      float error = q_des - q_act;

      char buf[160];
      int len = snprintf(buf, sizeof(buf), "LEG_SINE_LOG leg=%u m=%u t=", leg, motor);
      len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, t);
      if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " q_des=");
      len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, q_des);
      if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " q_act=");
      len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, q_act);
      if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " err=");
      len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, error);
      if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " dy=");
      len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, dy);
      if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");
      if (len > 0 && len < (int)sizeof(buf))
        Communication_SendString(buf);
    }
  }
}

void Leg_Gait_ServiceTrot(void)
{
  if (!trot.active)
    return;
  if (trot.profile_index >= LEG_TROT_PROFILE_COUNT) {
    Communication_SendString("LEG_TROT abort invalid_profile\r\n");
    trot.active = 0U;
    remote_armed = 0U;
    return;
  }
  const Leg_TrotProfileTypeDef *profile =
      &trot_profiles[trot.profile_index];
  uint32_t now = HAL_GetTick();
  Motor_GroupSnapshot group;
  Leg_Control_GetGroupSnapshot(&group);
  if (group.mode != Motor_Group_Active) {
    Communication_SendString("LEG_TROT abort group_inactive\r\n");
    trot.active = 0U;
    remote_armed = 0U;
    return;
  }

  float rotor_targets[8];
  float phase = 0.0f;
  float entry_blend = 1.0f;
  uint8_t enter_cycle = 0U;
  uint32_t phase_offset_ms = profile->half_cycle_ms / 2U;
  uint32_t cycle_time = 0U;

  if (trot.stage == 3U || trot.stage == 4U) {
    uint32_t elapsed = now - trot.stage_start_tick;
    float return_blend = trot.stage == 3U
                             ? (float)elapsed / (float)LEG_TROT_RETURN_MS
                             : 1.0f;
    if (return_blend > 1.0f) return_blend = 1.0f;
    if (return_blend < 0.0f) return_blend = 0.0f;
    float smooth = 0.5f - 0.5f * cosf(LEG_PI * return_blend);
    for (uint8_t i = 0U; i < 8U; ++i) {
      rotor_targets[i] = trot.return_start_targets[i] +
                         smooth * (trot.zero_targets[i] -
                                   trot.return_start_targets[i]);
      if (trot.stage == 4U) rotor_targets[i] = trot.zero_targets[i];
    }
    for (uint8_t leg = 0U; leg < 4U; ++leg) {
      trot.leg_state[leg] = trot.stage == 3U ? 'R' : 'H';
      if (!trot_rotor_targets_to_foot(leg, rotor_targets,
                                      &trot.target_foot[leg])) {
        Motor_GroupControl_StopWithContext(Motor_Group_StopController,
                                           (int8_t)(leg * 2U), smooth);
        trot.active = 0U;
        remote_armed = 0U;
        Communication_SendString("LEG_TROT abort return_fk\r\n");
        return;
      }
    }
    if (!set_trot_targets(rotor_targets)) {
      trot.active = 0U;
      remote_armed = 0U;
      Communication_SendString("LEG_TROT abort return_target_guard\r\n");
      return;
    }

    if ((now - trot.last_log_tick) >= LEG_TROT_LOG_PERIOD_MS) {
      trot.last_log_tick = now;
      capture_trot_telemetry(now, profile, 0.0f, smooth);
    }

    if (trot.stage == 3U && elapsed >= LEG_TROT_RETURN_MS) {
      trot.stage = 4U;
      trot.stage_start_tick = now;
      Communication_SendString(
          "LEG_TROT return_zero settle_internal_pd\r\n");
      return;
    }
    if (trot.stage == 4U && elapsed >= LEG_TROT_HOLD_SETTLE_MS) {
      uint32_t finish_primask = __get_PRIMASK();
      __disable_irq();
      int hold_finished = Motor_GroupControl_FinishGaitHold();
      if (finish_primask == 0U) __enable_irq();
      if (!hold_finished) {
        Motor_GroupControl_StopWithContext(Motor_Group_StopController,
                                           -1, -4.0f);
        trot.active = 0U;
        remote_armed = 0U;
        Communication_SendString("LEG_TROT abort hold_transition\r\n");
        return;
      }

      Motor_GroupDiagnostics diagnostics;
      Motor_GroupSnapshot result_group;
      Leg_Control_GetGroupDiagnostics(&diagnostics, 0U);
      Leg_Control_GetGroupSnapshot(&result_group);
      float p2p_max = 0.0f;
      float velocity_max = 0.0f;
      float error_max = 0.0f;
      for (uint8_t i = 0U; i < 8U; ++i) {
        if (diagnostics.position_peak_to_peak[i] > p2p_max)
          p2p_max = diagnostics.position_peak_to_peak[i];
        if (diagnostics.max_abs_velocity[i] > velocity_max)
          velocity_max = diagnostics.max_abs_velocity[i];
        if (fabsf(result_group.position_error[i]) > error_max)
          error_max = fabsf(result_group.position_error[i]);
      }
      trot.active = 0U;
      trot.stage = 0U;
      int len = snprintf(
          trot_result_line, sizeof(trot_result_line),
          "LEG_TROT_RESULT backend=internal_pd level=%u s2=%u dir=%d "
          "return_zero=1 hold=software_torque hold_mode=%u "
          "p2p_urad=%ld vmax_urad_s=%ld err_urad=%ld\r\n",
          (unsigned int)profile->level,
          (unsigned int)profile->ui_s2, (int)trot.direction,
          (unsigned int)result_group.mode,
          (long)(p2p_max * 1000000.0f),
          (long)(velocity_max * 1000000.0f),
          (long)(error_max * 1000000.0f));
      if (len > 0 && len < (int)sizeof(trot_result_line)) {
        trot_telemetry.pending = 0U;
        trot_result_pending = 1U;
      }
    }
    return;
  }

  if (trot.stage == 1U) {
    uint32_t elapsed = now - trot.stage_start_tick;
    float t = (float)elapsed / (float)profile->entry_ms;
    if (t > 1.0f) t = 1.0f;
    entry_blend = 0.5f - 0.5f * cosf(LEG_PI * t);
    cycle_time = (elapsed + phase_offset_ms) % profile->cycle_ms;
    phase = (float)cycle_time / (float)profile->cycle_ms;
    if (trot.stop_requested != 0U) {
      if (group.profile == Motor_Group_ProfileGait) {
        if (!begin_trot_return(
                now, "LEG_TROT entry_cancel smooth_return_zero\r\n")) {
          Communication_SendString("LEG_TROT abort entry_return_zero\r\n");
          trot.active = 0U;
          remote_armed = 0U;
          return;
        }
      } else {
        trot.active = 0U;
        trot.stage = 0U;
        trot.stop_requested = 0U;
        Communication_SendString("LEG_TROT entry_cancel zero_hold\r\n");
      }
      return;
    }
    if (elapsed >= profile->entry_ms) enter_cycle = 1U;
  } else {
    uint32_t elapsed = now - trot.stage_start_tick;
    cycle_time = (elapsed + phase_offset_ms) % profile->cycle_ms;
    phase = (float)cycle_time / (float)profile->cycle_ms;
    if ((trot.stop_requested || trot.one_cycle) &&
        trot.stop_after_tick == 0U) {
      uint32_t remaining_ms = profile->cycle_ms - cycle_time;
      trot.stop_after_tick = now + remaining_ms;
    }
    if (trot.stop_after_tick != 0U &&
        (int32_t)(now - trot.stop_after_tick) >= 0) {
      if (!begin_trot_return(
              now, "LEG_TROT finish_cycle smooth_return_zero\r\n")) {
        Communication_SendString("LEG_TROT abort return_zero\r\n");
        trot.active = 0U;
        remote_armed = 0U;
        return;
      }
      return;
    }
  }

  for (uint8_t leg = 0U; leg < 4U; ++leg) {
    uint8_t pair_a = (leg == 0U || leg == 3U) ? 1U : 0U;
    float leg_phase = phase + (pair_a != 0U ? 0.0f : 0.5f);
    if (leg_phase >= 1.0f) leg_phase -= 1.0f;
    uint8_t swing = leg_phase < profile->swing_fraction ? 1U : 0U;
    float t = swing != 0U
                  ? leg_phase / profile->swing_fraction
                  : (leg_phase - profile->swing_fraction) /
                        (1.0f - profile->swing_fraction);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float x;
    float y = profile->base_y_mm;
    if (swing) {
      float cycloid = t - sinf(LEG_TWO_PI * t) / LEG_TWO_PI;
      x = -profile->start_point_mm +
          2.0f * profile->start_point_mm * cycloid;
      y -= profile->lift_height_mm *
           (0.5f - 0.5f * cosf(LEG_TWO_PI * t));
    } else {
      x = profile->start_point_mm -
          2.0f * profile->start_point_mm * t;
    }
    x = LEG_TROT_BASE_X_MM + (float)trot.direction * x;
    if (trot.stage == 1U) {
      x = LEG_TROT_BASE_X_MM +
          entry_blend * (x - LEG_TROT_BASE_X_MM);
      y = LEG_TROT_ZERO_Y_MM +
          entry_blend * (y - LEG_TROT_ZERO_Y_MM);
    }

    trot.target_foot[leg].x = x;
    trot.target_foot[leg].y = y;
    trot.leg_state[leg] = swing ? 'S' : 'G';

    Leg_PointTypeDef foot = {.x = x, .y = y};
    Leg_JointAnglesTypeDef angles;
    float leg_targets[2];
    if (!Leg_Kinematics_Inverse(&foot, &angles) ||
        !Leg_Control_JointToRotorTargets(leg, &angles, leg_targets)) {
      Motor_GroupControl_StopWithContext(Motor_Group_StopController,
                                         (int8_t)(leg * 2U), y);
      trot.active = 0U;
      remote_armed = 0U;
      Communication_SendString("LEG_TROT abort ik\r\n");
      return;
    }
    rotor_targets[leg * 2U] = leg_targets[0];
    rotor_targets[leg * 2U + 1U] = leg_targets[1];
  }

  if (!set_trot_targets(rotor_targets)) {
    trot.active = 0U;
    remote_armed = 0U;
    Communication_SendString("LEG_TROT abort target_guard\r\n");
    return;
  }

  if (enter_cycle) {
    trot.stage = 2U;
    trot.stage_start_tick = now;
    if (trot.one_cycle) {
      uint32_t remaining_ms = profile->cycle_ms - phase_offset_ms;
      trot.stop_after_tick = now + remaining_ms;
    }
    Communication_SendString("LEG_TROT entry_complete cycle_start\r\n");
  }

  if ((now - trot.last_log_tick) >= LEG_TROT_LOG_PERIOD_MS) {
    trot.last_log_tick = now;
    capture_trot_telemetry(now, profile, phase, entry_blend);
  }
}

int Leg_Gait_StartDebugTrace(uint8_t leg)
{
  if (leg >= 4 || Leg_Gait_AnyActive())
    return 0;

  for (uint8_t motor = 0; motor < 2; motor++)
  {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    if (state->target_active == Motor_Target_Active)
      return 0;
  }

  Leg_PointTypeDef current;
  if (!Leg_Control_GetCurrentFoot(leg, &current))
    return 0;

  debug_trace.active = 1U;
  debug_trace.leg = leg;
  debug_trace.step = 0U;
  debug_trace.base_foot = current;
  log_trace_point("start", leg, 0U, &current, &current);
  return start_debug_trace_step();
}

int Leg_Gait_StartAllMicroTest(void)
{
  if (Leg_Gait_AnyActive())
    return 0;

  for (uint8_t idx = 0; idx < 8; idx++)
  {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state == NULL || state->target_active == Motor_Target_Active)
      return 0;
  }

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    if (!Leg_Control_GetCurrentFoot(leg, &all_micro.base_foot[leg]))
    {
      log_all_micro_simple("reject_fk", 0U, Motor_Target_Stopped);
      return 0;
    }
  }

  all_micro.active = 1U;
  all_micro.phase = 0U;
  if (!start_all_micro_phase(0U))
  {
    all_micro.active = 0U;
    return 0;
  }

  return 1;
}

int Leg_Gait_StartPrepPoseTest(void)
{
  if (Leg_Gait_AnyActive())
    return 0;

  for (uint8_t idx = 0; idx < 8; idx++)
  {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state == NULL || state->target_active == Motor_Target_Active)
      return 0;
  }

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    if (!Leg_Control_GetCurrentFoot(leg, &prep_pose.base_foot[leg]))
    {
      log_prep_pose_feet("reject_fk", Motor_Target_Stopped);
      return 0;
    }
  }

  prep_pose.active = 1U;
  if (!start_prep_pose_targets())
  {
    prep_pose.active = 0U;
    return 0;
  }

  return 1;
}

int Leg_Gait_StartSineTest(uint8_t leg, float amplitude_mm, float freq_hz)
{
  if (leg >= 4 || Leg_Gait_AnyActive())
    return 0;

  amplitude_mm = clampf(amplitude_mm, LEG_SINE_AMP_MIN_MM, LEG_SINE_AMP_MAX_MM);
  freq_hz = clampf(freq_hz, LEG_SINE_FREQ_MIN_HZ, LEG_SINE_FREQ_MAX_HZ);

  for (uint8_t idx = 0; idx < 8; idx++)
  {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state == NULL || state->target_active == Motor_Target_Active)
      return 0;
  }

  if (!Leg_Control_GetCurrentFoot(leg, &sine_test.base_foot))
    return 0;

  for (uint8_t motor = 0; motor < 2; motor++)
  {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    if (state->online != Motor_Online || state->angle_valid != Motor_Angle_Valid)
      return 0;
  }

  sine_test.active = 1U;
  sine_test.leg = leg;
  sine_test.amplitude_mm = amplitude_mm;
  sine_test.freq_hz = freq_hz;
  sine_test.start_tick = HAL_GetTick();
  sine_test.last_log_tick = 0;

  char buf[128];
  int len = snprintf(buf, sizeof(buf), "LEG_SINE start leg=%u amp=", leg);
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, amplitude_mm);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " hz=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, freq_hz);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);

  return 1;
}

int Leg_Gait_StartTrotTest(void)
{
  const Leg_TrotProfileTypeDef *profile = &trot_profiles[0];
  if (Leg_Gait_AnyActive())
    return 0;
  Motor_GroupSnapshot group;
  Leg_Control_GetGroupSnapshot(&group);
  if (group.mode != Motor_Group_Active || group.all_at_zero == 0U)
    return 0;
  if (!configure_trot_profile(profile))
    return 0;
  for (uint8_t i = 0U; i < 8U; ++i)
    trot.zero_targets[i] = group.target_position[i];
  trot.active = 1U;
  trot.stage = 1U;
  trot.profile_index = 0U;
  trot.direction = 1;
  trot.stop_requested = 0U;
  trot.one_cycle = 1U;
  trot.stage_start_tick = HAL_GetTick();
  trot.stop_after_tick = 0U;
  trot.last_log_tick = 0U;
  trot_telemetry.pending = 0U;
  trot_result_pending = 0U;
  char buf[288];
  int len = snprintf(buf, sizeof(buf),
                     "LEG_TROT start source=debug backend=internal_pd "
                     "owner=group_gait level=%u s2=%u dir=1 "
                     "lift=%d step=%d half_ms=%u entry_ms=%u "
                     "cycle_ms=%lu return_ms=%u kp_milli=%ld kw_milli=%ld "
                     "t_ff_mNm=0 w_mrad_s=0 xm_mrad=%d "
                     "path=cycloid_sine\r\n",
                     (unsigned int)profile->level,
                     (unsigned int)profile->ui_s2,
                     (int)profile->lift_height_mm,
                     (int)(profile->start_point_mm * 2.0f),
                     (unsigned int)profile->half_cycle_ms,
                     (unsigned int)profile->entry_ms,
                     (unsigned long)profile->cycle_ms,
                     (unsigned int)LEG_TROT_RETURN_MS,
                     (long)(LEG_TROT_INTERNAL_PD_KP * 1000.0f),
                     (long)(LEG_TROT_INTERNAL_PD_KW * 1000.0f),
                     (int)(profile->max_zero_excursion_rad * 1000.0f));
  if (len > 0 && len < (int)sizeof(buf)) Communication_SendString(buf);
  return 1;
}

void Leg_Gait_RemoteDisarm(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (trot.active) {
    if (Motor_GroupControl_ReturnGaitToZero())
      (void)Motor_GroupControl_FinishGaitHold();
  }
  remote_armed = 0U;
  trot.active = 0U;
  trot.stage = 0U;
  trot.stop_requested = 0U;
  trot.stop_after_tick = 0U;
  trot_telemetry.pending = 0U;
  if (primask == 0U) __enable_irq();
}

static int start_remote_trot(int8_t direction, uint8_t ui_s2)
{
  if (ui_s2 != LEG_TROT_INTERNAL_PD_UI_S2) {
    uint32_t now = HAL_GetTick();
    if ((now - remote_last_reject_log_tick) >= 500U) {
      remote_last_reject_log_tick = now;
      char reject[128];
      int len = snprintf(
          reject, sizeof(reject),
          "LEG_REMOTE gait_rejected internal_pd_only_s2=%u requested_s2=%u\r\n",
          (unsigned int)LEG_TROT_INTERNAL_PD_UI_S2,
          (unsigned int)ui_s2);
      if (len > 0 && len < (int)sizeof(reject))
        Communication_SendString(reject);
    }
    return 0;
  }
  uint8_t profile_index = trot_profile_index_from_s2(ui_s2);
  const Leg_TrotProfileTypeDef *profile = &trot_profiles[profile_index];
  Motor_GroupSnapshot group;
  Leg_Control_GetGroupSnapshot(&group);
  if (!remote_armed || direction == 0 || Leg_Gait_AnyActive() ||
      group.mode != Motor_Group_Active) {
    uint32_t now = HAL_GetTick();
    if ((now - remote_last_reject_log_tick) >= 500U) {
      remote_last_reject_log_tick = now;
      char reject[144];
      int len = snprintf(reject, sizeof(reject),
                         "LEG_REMOTE gait_rejected armed=%u dir=%d active=%u "
                         "mode=%u at_zero=%u\r\n",
                         (unsigned int)remote_armed, (int)direction,
                         (unsigned int)Leg_Gait_AnyActive(),
                         (unsigned int)group.mode,
                         (unsigned int)group.all_at_zero);
      if (len > 0 && len < (int)sizeof(reject))
        Communication_SendString(reject);
    }
    return 0;
  }
  /* configure_trot_profile() only accepts an active UniformOffset controller
   * whose commanded offset is zero.  That is the correct loaded-stand
   * precondition; all_at_zero is only an unloaded tracking diagnostic. */
  if (!configure_trot_profile(profile)) {
    Communication_SendString(
        "LEG_REMOTE gait_rejected profile_config rearm=stand\r\n");
    remote_armed = 0U;
    return 0;
  }
  for (uint8_t i = 0U; i < 8U; ++i)
    trot.zero_targets[i] = group.target_position[i];
  trot.active = 1U;
  trot.stage = 1U;
  trot.profile_index = profile_index;
  trot.direction = direction;
  trot.stop_requested = 0U;
  trot.one_cycle = 0U;
  trot.stage_start_tick = HAL_GetTick();
  trot.stop_after_tick = 0U;
  trot.last_log_tick = 0U;
  trot_telemetry.pending = 0U;
  trot_result_pending = 0U;
  char buf[352];
  int len = snprintf(buf, sizeof(buf),
                     "LEG_REMOTE gait_start backend=internal_pd "
                     "owner=group_gait dir=%d level=%u s2=%u "
                     "step=%d lift=%d "
                     "cycle_ms=%u entry_ms=%u return_ms=%u duty=%u "
                     "kp_milli=%ld kw_milli=%ld t_ff_mNm=0 w_mrad_s=0 "
                     "guard=feedback_fault_temp_range_owner xm_mrad=%d ref_s2=%u "
                     "path=cycloid_sine\r\n",
                     (int)direction,
                     (unsigned int)profile->level,
                     (unsigned int)profile->ui_s2,
                     (int)(profile->start_point_mm * 2.0f),
                     (int)profile->lift_height_mm,
                     (unsigned int)profile->cycle_ms,
                     (unsigned int)profile->entry_ms,
                     (unsigned int)LEG_TROT_RETURN_MS,
                     (unsigned int)((1.0f - profile->swing_fraction) *
                                    100.0f),
                     (long)(LEG_TROT_INTERNAL_PD_KP * 1000.0f),
                     (long)(LEG_TROT_INTERNAL_PD_KW * 1000.0f),
                     (int)(profile->max_zero_excursion_rad * 1000.0f),
                     (unsigned int)profile->ref_s2);
  if (len > 0 && len < (int)sizeof(buf)) Communication_SendString(buf);
  return 1;
}

void Leg_Gait_ServiceRemote(void)
{
  if (trot_result_pending != 0U) {
    if (Communication_TrySendString(trot_result_line))
      trot_result_pending = 0U;
  } else {
    service_trot_telemetry();
  }

  RC_DataTypeDef rc;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  rc = rc_data;
  if (primask == 0U) __enable_irq();

  uint8_t stand_edge =
      rc.s1 == 2U && rc.s2 == 1U &&
      (remote_last_s1 != 2U || remote_last_s2 != 1U);
  remote_last_s1 = rc.s1;
  remote_last_s2 = rc.s2;

  if (stand_edge) {
    if (trot.active) {
      trot.stop_requested = 1U;
      Communication_SendString("LEG_REMOTE stand queued_after_cycle\r\n");
    } else if (Leg_Control_ArmAllZero() && Leg_Control_StartAllZero()) {
      remote_armed = 1U;
      Communication_SendString("LEG_REMOTE stand zero_target hold=1 armed=1\r\n");
    } else {
      remote_armed = 0U;
      Communication_SendString("LEG_REMOTE stand rejected\r\n");
    }
  }

  int8_t requested_direction = 0;
  if (rc.s1 == 3U) {
    /* The web joystick reports upward/forward motion below 1024, so its
     * centered ch3 is negative when the operator pushes forward. */
    if (rc.ch3 < -LEG_REMOTE_CHANNEL_DEADZONE)
      requested_direction = 1;
    else if (rc.ch3 > LEG_REMOTE_CHANNEL_DEADZONE)
      requested_direction = -1;
  }

  if (trot.active) {
    if (trot.profile_index < LEG_TROT_PROFILE_COUNT &&
        rc.s1 == 3U && rc.s2 >= 1U && rc.s2 <= 3U &&
        trot.profile_index != trot_profile_index_from_s2(rc.s2) &&
        trot.stop_requested == 0U) {
      char level_log[96];
      int len = snprintf(level_log, sizeof(level_log),
                         "LEG_REMOTE level_change queued from=%u to=%u\r\n",
                         (unsigned int)trot_profiles[trot.profile_index].ui_s2,
                         (unsigned int)rc.s2);
      if (len > 0 && len < (int)sizeof(level_log))
        Communication_SendString(level_log);
      trot.stop_requested = 1U;
    }
    if (!Communication_IsLinkAlive()) {
      if (remote_armed != 0U)
        Communication_SendString(
            "LEG_REMOTE link_lost stop_after_cycle rearm_required=1\r\n");
      remote_armed = 0U;
      trot.stop_requested = 1U;
    } else if (requested_direction == 0 ||
               requested_direction != trot.direction) {
      trot.stop_requested = 1U;
    }
  } else if (requested_direction != 0) {
    (void)start_remote_trot(requested_direction, rc.s2);
  }

  Leg_Gait_ServiceTrot();
}
