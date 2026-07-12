#include "Leg_Gait.h"
#include "Leg_Kinematics.h"
#include "Leg_Control.h"
#include "communication.h"
#include "GO-M8010-6.h"
#include <math.h>
#include <stdio.h>

extern Leg_HandlerTypeDef* Legs[4];

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
#define LEG_TROT_LIFT_HEIGHT_MM     80.0f
#define LEG_TROT_START_POINT_MM     50.0f
#define LEG_TROT_STEP_RATE_MS       400U
#define LEG_TROT_STEP_CYCLE_MS      800U
#define LEG_TROT_LOG_PERIOD_MS      500U
#define LEG_TROT_KP                 6.0f
#define LEG_TROT_KW                 0.35f

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
  uint8_t active;
  uint32_t start_tick;
  uint32_t last_log_tick;
  float leg_high[4];
} Leg_TrotTypeDef;

static Leg_DebugTraceTypeDef debug_trace = {0};
static Leg_AllMicroTypeDef all_micro = {0};
static Leg_PrepPoseTypeDef prep_pose = {0};
static Leg_SineTypeDef sine_test = {0};
static Leg_TrotTypeDef trot = {0};

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
  debug_trace.active = 0U;
  all_micro.active = 0U;
  prep_pose.active = 0U;
  sine_test.active = 0U;
  trot.active = 0U;
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

  for (uint8_t i = 0; i < 8; i++)
  {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(i);
    if (state->online != Motor_Online || state->angle_valid != Motor_Angle_Valid)
    {
      Communication_SendString("LEG_TROT abort offline\r\n");
      trot.active = 0U;
      return;
    }
  }

  uint32_t now = HAL_GetTick();
  uint32_t cycle_time = (now - trot.start_tick) % LEG_TROT_STEP_CYCLE_MS;
  uint8_t half_cycle = (cycle_time >= LEG_TROT_STEP_RATE_MS) ? 1U : 0U;
  uint32_t phase_ms = half_cycle ? (cycle_time - LEG_TROT_STEP_RATE_MS) : cycle_time;
  float t = (float)phase_ms / (float)LEG_TROT_STEP_RATE_MS;
  if (t > 1.0f) t = 1.0f;

  uint8_t pair_a_swing = half_cycle;
  float foot_x_saved[4];
  float foot_y_saved[4];

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    uint8_t in_pair_a = (leg == 0U || leg == 3U) ? 1U : 0U;
    uint8_t is_swing = in_pair_a ? pair_a_swing : (1U - pair_a_swing);

    float foot_x, foot_y;
    if (is_swing)
    {
      float len = 2.0f * LEG_TROT_START_POINT_MM;
      foot_x = -LEG_TROT_START_POINT_MM + len * (t - sinf(LEG_TWO_PI * t) / LEG_TWO_PI);
      foot_y = trot.leg_high[leg] - LEG_TROT_LIFT_HEIGHT_MM * (0.5f - 0.5f * cosf(LEG_TWO_PI * t));
    }
    else
    {
      foot_x = LEG_TROT_START_POINT_MM - 2.0f * LEG_TROT_START_POINT_MM * t;
      foot_y = trot.leg_high[leg];
    }

    foot_x_saved[leg] = foot_x;
    foot_y_saved[leg] = foot_y;

    Leg_PointTypeDef target = { .x = foot_x, .y = foot_y };
    Leg_JointAnglesTypeDef target_angles;
    if (!Leg_Kinematics_Inverse(&target, &target_angles))
      continue;

    float rotor_targets[2];
    if (!Leg_Control_JointToRotorTargets(leg, &target_angles, rotor_targets))
      continue;

    for (uint8_t motor = 0; motor < 2; motor++)
    {
      float rotor_target = rotor_targets[motor];
      MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
      cmd->mode = 1;
      cmd->T = 0.0f;
      cmd->W = 0.0f;
      cmd->Pos = rotor_target;
      cmd->K_P = LEG_TROT_KP;
      cmd->K_W = LEG_TROT_KW;
      modify_data(cmd);
    }
  }

  if ((now - trot.last_log_tick) >= LEG_TROT_LOG_PERIOD_MS)
  {
    trot.last_log_tick = now;
    char buf[320];
    int len = snprintf(buf, sizeof(buf), "LEG_TROT_LOG hc=%u t=", half_cycle);
    if (len > 0 && len < (int)sizeof(buf))
    {
      char *bp = buf + len;
      size_t rem = sizeof(buf) - (size_t)len;
      int n = snprintf(bp, rem, "%ld.%03ld", (long)((int)t), (long)((int)(t * 1000) % 1000));
      if (n > 0 && n < (int)rem) { bp += n; rem -= n; } else rem = 0;
      for (uint8_t lg = 0; lg < 4 && rem > 10; lg++)
      {
        uint8_t in_pair_a = (lg == 0U || lg == 3U) ? 1U : 0U;
        uint8_t is_swing = in_pair_a ? pair_a_swing : (1U - pair_a_swing);
        n = snprintf(bp, rem, " L%u:%c(%ld,%ld)",
                     lg, is_swing ? 'S' : 'G',
                     (long)(int)foot_x_saved[lg], (long)(int)foot_y_saved[lg]);
        if (n > 0 && n < (int)rem) { bp += n; rem -= n; }
      }
      if (rem > 2) { *bp++ = '\r'; *bp++ = '\n'; rem -= 2; }
      *bp = '\0';
      Communication_SendString(buf);
    }
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
    Leg_PointTypeDef foot;
    if (!Leg_Control_GetCurrentFoot(leg, &foot))
      return 0;
    trot.leg_high[leg] = foot.y;
  }

  trot.active = 1U;
  trot.start_tick = HAL_GetTick();
  trot.last_log_tick = 0;

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "LEG_TROT start lift=%d step=%d rate=%dms kp=%d kw=%d\r\n",
                     (int)LEG_TROT_LIFT_HEIGHT_MM,
                     (int)(LEG_TROT_START_POINT_MM * 2.0f),
                     (int)LEG_TROT_STEP_RATE_MS,
                     (int)(LEG_TROT_KP * 1000),
                     (int)(LEG_TROT_KW * 1000));
  if (len > 0 && len < (int)sizeof(buf))
  {
    buf[len] = '\0';
    Communication_SendString(buf);
  }

  return 1;
}
