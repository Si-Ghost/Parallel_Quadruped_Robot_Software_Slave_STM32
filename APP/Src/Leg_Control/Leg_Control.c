#include "Leg_Control.h"
#include "Motor_Transport.h"
#include "Leg_Gait.h"
#include "communication.h"
#include <math.h>
#include <stdio.h>

extern Leg_HandlerTypeDef Left_Front_Leg;
extern Leg_HandlerTypeDef Right_Front_Leg;
extern Leg_HandlerTypeDef Left_Back_Leg;
extern Leg_HandlerTypeDef Right_Back_Leg;

extern Leg_HandlerTypeDef* Legs[4];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

#define LEG_SERVICE_PERIOD_MS       50U
#define LEG_WEB_ANGLE_MIN_RAD      -1.50f
#define LEG_WEB_ANGLE_MAX_RAD       1.50f
#define LEG_WEB_STOP_ERROR_RAD      0.08f
#define LEG_WEB_NEAR_ERROR_RAD      0.12f
#define LEG_WEB_TARGET_TIMEOUT_MS   30000U
#define LEG_WEB_NO_PROGRESS_MS      4000U
#define LEG_WEB_STALL_GRACE_MS      500U
#define LEG_WEB_STALL_PROGRESS_RAD  0.005f
#define LEG_FOOT_NUDGE_MAX_MM       15.0f
#define LEG_FOOT_NUDGE_ROTOR_RAD    0.65f
#define LEG_HANDSHAKE_KW            0.05f
#define LEG_HANDSHAKE_RETRY         2U
#define LEG_DEBUG_LOG_PERIOD_MS     500U
#define LEG_IO_ERROR_LOG_PERIOD_MS  1000U
#define LEG_IO_ERROR_OFFLINE_COUNT  5U
#define LEG_IO_RETRY_COUNT          1U
#define LEG_ROTOR_ZERO_NEAR_RAD     1.50f
#define LEG_MOTOR_THETA1            0U /* ID0 drives AB after front/rear mounting swap. */
#define LEG_MOTOR_THETA2            1U /* ID1 drives AD after front/rear mounting swap. */

static uint32_t last_service_tick = 0;
static volatile uint8_t handshake_requested = 0;

static void refresh_leg_online_state(uint8_t leg);

static const float default_rotor_zero_offset[4][2] = {
  /* LF: ID0, ID1 */ { 4.4221f, 5.5321f},
  /* RF: ID0, ID1 */ {-0.7610f, 4.5657f},
  /* LB: ID0, ID1 */ { 0.0832f, 3.3468f},
  /* RB: ID0, ID1 */ {-0.9794f, 3.3552f},
};

static const float default_motor_direction[4][2] = {
  /* LF: ID0, ID1 */ { 1.0f, 1.0f},
  /* RF: ID0, ID1 */ { 1.0f, 1.0f},
  /* LB: ID0, ID1 */ { 1.0f, 1.0f},
  /* RB: ID0, ID1 */ { 1.0f, 1.0f},
};

/* ---- helpers ---- */

static uint8_t leg_index_from_handle(Leg_HandlerTypeDef *hleg)
{
  for (uint8_t leg = 0; leg < 4; leg++)
    if (Legs[leg] == hleg) return leg;
  return 0xFF;
}

uint8_t Leg_Control_MotorIndex(uint8_t leg, uint8_t motor)
{
  return leg * 2U + motor;
}

static Motor_RuntimeStateTypeDef *motor_state_from_leg_motor(uint8_t leg, uint8_t motor)
{
  if (leg >= 4 || motor >= 2) return NULL;
  return &Legs[leg]->motor_state[motor];
}

Motor_RuntimeStateTypeDef *Leg_Control_MotorState(uint8_t motor_index)
{
  if (motor_index >= 8) return NULL;
  return motor_state_from_leg_motor(motor_index / 2U, motor_index % 2U);
}

float Leg_Control_NormalizedMotorDir(float direction)
{
  return (direction < 0.0f) ? -1.0f : 1.0f;
}

static float joint_to_rotor_angle(uint8_t leg, uint8_t motor, float joint_angle)
{
  if (leg >= 4U || motor >= 2U) return 0.0f;
  const Leg_HandlerTypeDef *hleg = Legs[leg];
  const Motor_RuntimeStateTypeDef *state = &hleg->motor_state[motor];
  float direction = Leg_Control_NormalizedMotorDir(hleg->motor_direction[motor]);
  float zero_reference = state->zero_reference_valid
                             ? state->zero_rotor_position
                             : hleg->rotor_zero_offset[motor];
  return zero_reference + direction * joint_angle * LEG_REDUCTION_RATIO;
}

static void reset_motor_runtime_state(Motor_RuntimeStateTypeDef *state,
                                      float zero_offset,
                                      float direction)
{
  if (state == NULL) return;
  uint32_t error_count = state->error_count;
  Motor_State_Init(state, zero_offset, direction);
  state->error_count = error_count;
  state->target_offset = 0.0f;
  state->target_active = Motor_Target_Inactive;
  state->target_result = Motor_Target_Idle;
  state->target_start_angle = 0.0f;
  state->target_start_tick = 0;
  state->target_progress_tick = 0;
  state->target_last_abs_error = 0.0f;
  state->target_stop_error = LEG_WEB_STOP_ERROR_RAD;
  state->debug_last_log_tick = 0;
  state->io_error_last_log_tick = 0;
  state->io_error_count = 0;
}

static uint8_t transport_load_command(uint8_t leg, uint8_t motor, MOTOR_send *command)
{
  if (leg >= 4U || motor >= 2U || command == NULL) return 0U;
  *command = Legs[leg]->motor_cmd[motor];
  return 1U;
}

static void transport_feedback_received(uint8_t leg,
                                        uint8_t motor,
                                        const MOTOR_recv *feedback,
                                        uint32_t timestamp)
{
  if (leg >= 4U || motor >= 2U || feedback == NULL) return;
  Legs[leg]->motor_data[motor] = *feedback;
  Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
  Motor_State_UpdateRawFeedback(state, feedback->Pos, feedback->W, feedback->T,
                                timestamp, LEG_REDUCTION_RATIO,
                                LEG_ROTOR_ZERO_NEAR_RAD);
  state->handshake_status = Motor_Handshake_Ok;
  state->io_error_count = 0U;
  refresh_leg_online_state(leg);
}

static void transport_feedback_timeout(uint8_t leg, uint8_t motor, uint32_t timestamp)
{
  (void)timestamp;
  if (leg >= 4U || motor >= 2U) return;
  Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
  Motor_State_RecordError(state);
  Motor_State_MarkOffline(state);
  state->handshake_status = Motor_Handshake_Timeout;
  refresh_leg_online_state(leg);
}

static void transport_uart_error(uint8_t leg, uint32_t error_bits, uint32_t timestamp)
{
  (void)error_bits;
  (void)timestamp;
  if (leg >= 4U) return;
  for (uint8_t motor = 0U; motor < 2U; ++motor) {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    Motor_State_RecordError(state);
    state->handshake_status = Motor_Handshake_UartError;
  }
}

static const Motor_TransportCallbacks motor_transport_callbacks = {
  .load_command = transport_load_command,
  .feedback_received = transport_feedback_received,
  .feedback_timeout = transport_feedback_timeout,
  .uart_error = transport_uart_error,
};

static uint8_t motor_is_online(uint8_t leg, uint8_t motor)
{
  Motor_RuntimeStateTypeDef *state = motor_state_from_leg_motor(leg, motor);
  if (state == NULL) return 0;
  return state->online != 0;
}

static Leg_StatusTypeDef tx_status_for_motor(uint8_t motor)
{
  return (motor == 0) ? Leg_TX_M0 : Leg_TX_M1;
}

int Leg_Control_AppendFixed4(char *buf, size_t size, int pos, float value)
{
  if (pos < 0 || (size_t)pos >= size) return -1;
  int32_t scaled;
  if (value >= 0.0f)
    scaled = (int32_t)(value * 10000.0f + 0.5f);
  else
    scaled = (int32_t)(value * 10000.0f - 0.5f);
  const char *sign = "";
  if (scaled < 0) { sign = "-"; scaled = -scaled; }
  int written = snprintf(&buf[pos], size - (size_t)pos, "%s%ld.%04ld",
                         sign, (long)(scaled / 10000), (long)(scaled % 10000));
  if (written < 0 || written >= (int)(size - (size_t)pos)) return -1;
  return pos + written;
}

/* ---- debug target engine ---- */

static void log_leg_finish_if_idle(uint8_t leg, Motor_TargetResultTypeDef result);

static void log_debug_target(uint8_t motor_index, const char *tag, float desired, float delta)
{
  if (motor_index >= 8) return;
  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
  Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(motor_index);

  char buf[160];
  int len = snprintf(buf, sizeof(buf), "MOTOR_CMD %s i=%u off=", tag, motor_index);
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, state->target_offset);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " spd=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, cmd->W);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " tgt=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, desired);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " fbk=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, state->angle);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " d=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, delta);
  if (len > 0) {
    int written = snprintf(&buf[len], sizeof(buf) - (size_t)len,
                           " rt=%d rpos=%ld rspd=%d kp=%d kw=%d act=%u res=%u\r\n",
                           (int)cmd->motor_send_data.comd.tor_des,
                           (long)cmd->motor_send_data.comd.pos_des,
                           (int)cmd->motor_send_data.comd.spd_des,
                           (int)cmd->motor_send_data.comd.k_pos,
                           (int)cmd->motor_send_data.comd.k_spd,
                           state->target_active, state->target_result);
    if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len)) return;
    len += written;
  }
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

void Leg_Control_StopDebugTarget(uint8_t motor_index, Motor_TargetResultTypeDef result)
{
  if (motor_index >= 8) return;
  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  uint8_t paired_motor = motor ^ 1U;
  uint8_t keep_leg_damping =
      (result == Motor_Target_Done &&
       Legs[leg]->motor_state[paired_motor].target_active == Motor_Target_Active) ? 1U : 0U;
  uint8_t hold_position = (result == Motor_Target_Done) ? 1U : 0U;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
  Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(motor_index);

  state->target_active = Motor_Target_Inactive;
  state->target_result = result;
  state->target_start_angle = state->angle;
  state->target_start_tick = 0;
  state->target_progress_tick = 0;
  state->target_last_abs_error = 0.0f;
  state->target_stop_error = LEG_WEB_STOP_ERROR_RAD;

  cmd->mode = 1;
  cmd->T = 0.0f;
  cmd->W = 0.0f;
  cmd->K_P = hold_position ? LEG_HOLD_KP : 0.0f;
  cmd->K_W = hold_position ? LEG_HOLD_KW : (keep_leg_damping ? LEG_WEB_KW : 0.0f);
  cmd->Pos = hold_position ? (Legs[leg]->p_init[motor] + state->target_offset) : state->angle;
  modify_data(cmd);
  log_leg_finish_if_idle(leg, result);
}

static void log_leg_foot_line(const char *tag, uint8_t leg, const Leg_PointTypeDef *foot,
                              Motor_TargetResultTypeDef result)
{
  if (tag == NULL || foot == NULL || leg >= 4) return;
  char buf[128];
  int len = snprintf(buf, sizeof(buf), "LEG_FOOT %s leg=%u x=", tag, leg);
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, foot->x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " y=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, foot->y);
  if (len > 0) {
    int written = snprintf(&buf[len], sizeof(buf) - (size_t)len, " res=%u\r\n", result);
    if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len)) return;
    len += written;
  }
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void log_leg_finish_if_idle(uint8_t leg, Motor_TargetResultTypeDef result)
{
  if (leg >= 4) return;
  if (Legs[leg]->motor_state[0].target_active != Motor_Target_Inactive ||
      Legs[leg]->motor_state[1].target_active != Motor_Target_Inactive)
    return;
  if (result != Motor_Target_Done && result != Motor_Target_Timeout && result != Motor_Target_Stall)
    return;

  for (uint8_t motor = 0; motor < 2; motor++) {
    MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
    cmd->mode = 1;
    cmd->T = 0.0f;
    cmd->W = 0.0f;
    cmd->K_P = (result == Motor_Target_Done) ? LEG_HOLD_KP : 0.0f;
    cmd->K_W = (result == Motor_Target_Done) ? LEG_HOLD_KW : 0.0f;
    cmd->Pos = (result == Motor_Target_Done)
                   ? (Legs[leg]->p_init[motor] + Legs[leg]->motor_state[motor].target_offset)
                   : Legs[leg]->motor_state[motor].angle;
    modify_data(cmd);
  }

  Leg_JointAnglesTypeDef angles;
  uint8_t valid[2] = {0, 0};
  Leg_PointTypeDef foot;
  if (Leg_Control_GetJointAngles(leg, &angles, valid) &&
      valid[0] && valid[1] &&
      Leg_Kinematics_Forward(&angles, &foot))
    log_leg_foot_line("finish", leg, &foot, result);
}

int Leg_Control_ApplyDebugTarget(uint8_t motor_index, uint8_t force_log)
{
  Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(motor_index);
  if (state == NULL || !state->online) return 0;

  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];

  float final_desired = Legs[leg]->p_init[motor] + state->target_offset;
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - state->target_start_tick;
  float ramp = (float)elapsed / 1200.0f; /* LEG_TARGET_RAMP_MS */
  ramp = clampf(ramp, 0.0f, 1.0f);
  float desired = state->target_start_angle + (final_desired - state->target_start_angle) * ramp;
  float error = final_desired - state->angle;
  float abs_error = absf_local(error);
  float stop_error = (state->target_stop_error > 0.0f) ? state->target_stop_error : LEG_WEB_STOP_ERROR_RAD;

  if (state->target_active) {
    if (ramp >= 1.0f && abs_error <= stop_error) {
      Leg_Control_StopDebugTarget(motor_index, Motor_Target_Done);
    } else if ((now - state->target_start_tick) >= LEG_WEB_TARGET_TIMEOUT_MS) {
      Leg_Control_StopDebugTarget(motor_index, Motor_Target_Timeout);
    } else if ((state->target_last_abs_error - abs_error) >= LEG_WEB_STALL_PROGRESS_RAD) {
      state->target_last_abs_error = abs_error;
      state->target_progress_tick = now;
    } else if ((now - state->target_start_tick) >= LEG_WEB_STALL_GRACE_MS &&
               (now - state->target_progress_tick) >= LEG_WEB_NO_PROGRESS_MS) {
      Leg_Control_StopDebugTarget(motor_index, Motor_Target_Stall);
    }
    if (state->target_active == Motor_Target_Inactive) {
      if (force_log || (now - state->debug_last_log_tick) >= LEG_DEBUG_LOG_PERIOD_MS) {
        state->debug_last_log_tick = now;
        log_debug_target(motor_index, "done", desired, error);
      }
      return 1;
    }
  }

  cmd->mode = 1;
  cmd->T = 0.0f;
  cmd->W = 0.0f;
  cmd->Pos = state->target_active ? desired : state->angle;
  cmd->K_P = state->target_active ? LEG_WEB_KP : 0.0f;
  cmd->K_W = state->target_active ? LEG_WEB_KW : 0.0f;
  modify_data(cmd);

  if (force_log || !state->target_active ||
      (now - state->debug_last_log_tick) >= LEG_DEBUG_LOG_PERIOD_MS) {
    state->debug_last_log_tick = now;
    log_debug_target(motor_index, state->target_active ? "step" : "done", desired, error);
  }
  return 1;
}

static void __attribute__((unused)) update_debug_targets(void)
{
  for (uint8_t idx = 0; idx < 8; idx++) {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state != NULL && state->target_active)
      Leg_Control_ApplyDebugTarget(idx, 0);
  }
}

void Leg_Control_StartOffset(uint8_t motor_index, float offset)
{
  Leg_Control_StartOffsetWithStopError(motor_index, offset, LEG_WEB_STOP_ERROR_RAD);
}

void Leg_Control_StartOffsetWithStopError(uint8_t motor_index, float offset, float stop_error)
{
  Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(motor_index);
  if (state == NULL) return;
  state->target_offset = offset;
  state->target_active = Motor_Target_Active;
  state->target_result = Motor_Target_Running;
  state->target_start_angle = state->angle;
  state->target_start_tick = HAL_GetTick();
  state->target_progress_tick = state->target_start_tick;
  state->target_last_abs_error =
      absf_local((Legs[motor_index / 2]->p_init[motor_index % 2] + state->target_offset) - state->angle);
  state->target_stop_error = (stop_error > 0.0f) ? stop_error : LEG_WEB_STOP_ERROR_RAD;
}

/* ---- foot-level helpers ---- */

int Leg_Control_GetCurrentFoot(uint8_t leg, Leg_PointTypeDef *foot)
{
  if (leg >= 4 || foot == NULL) return 0;
  Leg_JointAnglesTypeDef angles;
  uint8_t valid[2] = {0, 0};
  return Leg_Control_GetJointAngles(leg, &angles, valid) &&
         valid[0] && valid[1] &&
         Leg_Kinematics_Forward(&angles, foot);
}

int Leg_Control_ComputeFootTargetOffsets(uint8_t leg, const Leg_PointTypeDef *target_foot, float offsets[2])
{
  if (leg >= 4 || target_foot == NULL || offsets == NULL) return 0;

  for (uint8_t motor = 0; motor < 2; motor++) {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    if (state->online != Motor_Online || state->angle_valid != Motor_Angle_Valid) return 0;
    float home_zero_error = Motor_State_GetZeroError(state);
    if (absf_local(home_zero_error) > LEG_ROTOR_ZERO_NEAR_RAD) return 0;
  }

  Leg_JointAnglesTypeDef target_angles;
  if (!Leg_Kinematics_Inverse(target_foot, &target_angles)) return 0;

  float rotor_targets[2];
  if (!Leg_Control_JointToRotorTargets(leg, &target_angles, rotor_targets)) return 0;

  for (uint8_t motor = 0; motor < 2; motor++) {
    offsets[motor] = rotor_targets[motor] - Legs[leg]->p_init[motor];
    if (absf_local(offsets[motor]) > LEG_FOOT_NUDGE_ROTOR_RAD) return 0;
  }
  return 1;
}

/* ---- nudge / foot offset ---- */

static void log_leg_nudge_reject(uint8_t leg, const char *reason, uint8_t motor, float value)
{
  char buf[128];
  int len = snprintf(buf, sizeof(buf), "LEG_NUDGE reject leg=%u reason=%s motor=%u value=",
                     leg, reason, motor);
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, value);
  if (len > 0) {
    int written = snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");
    if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len)) return;
    len += written;
  }
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

int Leg_Control_SetDebugFootOffset(uint8_t leg, float dx_mm, float dy_mm)
{
  if (leg >= 4 || Leg_Gait_AnyActive()) return 0;

  dx_mm = clampf(dx_mm, -LEG_FOOT_NUDGE_MAX_MM, LEG_FOOT_NUDGE_MAX_MM);
  dy_mm = clampf(dy_mm, -LEG_FOOT_NUDGE_MAX_MM, LEG_FOOT_NUDGE_MAX_MM);

  for (uint8_t motor = 0; motor < 2; motor++) {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    if (state->online != Motor_Online) { log_leg_nudge_reject(leg, "offline", motor, 0.0f); return 0; }
    if (state->angle_valid != Motor_Angle_Valid) { log_leg_nudge_reject(leg, "angle_invalid", motor, 0.0f); return 0; }
    float home_zero_error = Motor_State_GetZeroError(state);
    if (absf_local(home_zero_error) > LEG_ROTOR_ZERO_NEAR_RAD) {
      log_leg_nudge_reject(leg, "home_zero_out", motor, home_zero_error);
      return 0;
    }
  }

  Leg_JointAnglesTypeDef current_angles;
  uint8_t theta_valid[2] = {0, 0};
  if (!Leg_Control_GetJointAngles(leg, &current_angles, theta_valid) || !theta_valid[0] || !theta_valid[1]) {
    log_leg_nudge_reject(leg, "theta_invalid", 0U, 0.0f); return 0;
  }

  Leg_PointTypeDef current_foot;
  if (!Leg_Kinematics_Forward(&current_angles, &current_foot)) {
    log_leg_nudge_reject(leg, "fk_fail", 0U, 0.0f); return 0;
  }

  Leg_PointTypeDef target_foot = { .x = current_foot.x + dx_mm, .y = current_foot.y + dy_mm };
  Leg_JointAnglesTypeDef target_angles;
  if (!Leg_Kinematics_Inverse(&target_foot, &target_angles)) {
    log_leg_nudge_reject(leg, "ik_fail", 0U, target_foot.y); return 0;
  }

  float rotor_targets[2];
  if (!Leg_Control_JointToRotorTargets(leg, &target_angles, rotor_targets)) {
    log_leg_nudge_reject(leg, "rotor_target_fail", 0U, 0.0f); return 0;
  }

  float offsets[2];
  for (uint8_t motor = 0; motor < 2; motor++) {
    offsets[motor] = rotor_targets[motor] - Legs[leg]->p_init[motor];
    if (absf_local(offsets[motor]) > LEG_FOOT_NUDGE_ROTOR_RAD) {
      log_leg_nudge_reject(leg, "rotor_offset_limit", motor, offsets[motor]); return 0;
    }
  }

  char buf[160];
  int len = snprintf(buf, sizeof(buf), "LEG_FOOT plan leg=%u cur=", leg);
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, current_foot.x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, current_foot.y);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " tgt=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, target_foot.x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, target_foot.y);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " off=");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, offsets[0]);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, offsets[1]);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);

  Leg_Control_StartOffset(Leg_Control_MotorIndex(leg, 0), offsets[0]);
  Leg_Control_StartOffset(Leg_Control_MotorIndex(leg, 1), offsets[1]);
  int ok0 = Leg_Control_ApplyDebugTarget(Leg_Control_MotorIndex(leg, 0), 1);
  int ok1 = Leg_Control_ApplyDebugTarget(Leg_Control_MotorIndex(leg, 1), 1);
  return ok0 && ok1;
}

/* ---- single-motor angle test ---- */

int Leg_Control_SetDebugAngle(uint8_t motor_index, float angle_rad)
{
  if (Leg_Gait_AnyActive()) return 0;
  Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(motor_index);
  if (state == NULL || !state->online) return 0;
  state->target_offset = clampf(angle_rad, LEG_WEB_ANGLE_MIN_RAD, LEG_WEB_ANGLE_MAX_RAD);
  state->target_active = Motor_Target_Active;
  state->target_result = Motor_Target_Running;
  state->target_start_angle = state->angle;
  state->target_start_tick = HAL_GetTick();
  state->target_progress_tick = state->target_start_tick;
  state->target_last_abs_error =
      absf_local((Legs[motor_index / 2]->p_init[motor_index % 2] + state->target_offset) - state->angle);
  state->target_stop_error = LEG_WEB_STOP_ERROR_RAD;
  return Leg_Control_ApplyDebugTarget(motor_index, 1);
}

/* ---- IO error handling ---- */

static void log_motor_io_error(uint8_t motor_index, HAL_StatusTypeDef ret, MOTOR_recv *fbk)
{
  if (motor_index >= 8) return;
  uint32_t now = HAL_GetTick();
  Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(motor_index);
  if ((now - state->io_error_last_log_tick) < LEG_IO_ERROR_LOG_PERIOD_MS) return;
  state->io_error_last_log_tick = now;

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "MOTOR_IO fail idx=%u ret=%d rxlen=%u ok=%d err=%u cnt=%u h=%02X%02X id=%u\r\n",
                     motor_index, (int)ret, (unsigned int)fbk->rx_len, fbk->correct,
                     fbk->MError, state->io_error_count,
                     fbk->motor_recv_data.head[0], fbk->motor_recv_data.head[1],
                     fbk->motor_recv_data.mode.id);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void refresh_leg_online_state(uint8_t leg)
{
  if (leg >= 4) return;
  Legs[leg]->has_online_motor =
      (motor_is_online(leg, 0) || motor_is_online(leg, 1)) ? Motor_Online : Motor_Offline;
}

static void leg_abort_transfer(Leg_HandlerTypeDef *hleg)
{
  if (hleg == NULL) return;
  if (hleg->huartx != NULL) HAL_UART_Abort(hleg->huartx);
  hleg->Leg_Status = Leg_Idle;
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_RESET);
}

static void handle_motor_io_failure(uint8_t leg, uint8_t motor, HAL_StatusTypeDef ret, MOTOR_recv *fbk)
{
  if (leg >= 4 || motor >= 2) return;
  uint8_t idx = Leg_Control_MotorIndex(leg, motor);
  Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];

  if (state->io_error_count < 255U) state->io_error_count++;
  Motor_State_RecordError(state);
  state->angle_valid = Motor_Angle_Invalid;
  state->zero_checked = 0U;
  log_motor_io_error(idx, ret, fbk);

  if (state->io_error_count < LEG_IO_ERROR_OFFLINE_COUNT) return;

  Leg_Control_StopDebugTarget(idx, Motor_Target_Stopped);
  Motor_State_MarkOffline(state);
  state->handshake_status = (ret == HAL_TIMEOUT) ? Motor_Handshake_Timeout : Motor_Handshake_UartError;
  refresh_leg_online_state(leg);
  leg_abort_transfer(Legs[leg]);

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "MOTOR_IO offline idx=%u ret=%d rxlen=%u cnt=%u h=%02X%02X id=%u rescan required\r\n",
                     idx, (int)ret, (unsigned int)fbk->rx_len, state->io_error_count,
                     fbk->motor_recv_data.head[0], fbk->motor_recv_data.head[1],
                     fbk->motor_recv_data.mode.id);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static uint8_t motor_feedback_is_valid(HAL_StatusTypeDef ret, MOTOR_recv *fbk, uint8_t motor)
{
  return (ret == HAL_OK && fbk->correct && fbk->motor_id == motor) ? 1U : 0U;
}

/* ---- init & handshake ---- */

void Leg_Control_InitSafe(void)
{
  Left_Front_Leg.GPIOx = Left_Front_Leg_Control_GPIO_Port;
  Left_Front_Leg.GPIO_Pin = Left_Front_Leg_Control_Pin;
  Left_Front_Leg.huartx = &huart2;

  Right_Front_Leg.GPIOx = Right_Front_Leg_Control_GPIO_Port;
  Right_Front_Leg.GPIO_Pin = Right_Front_Leg_Control_Pin;
  Right_Front_Leg.huartx = &huart8;

  Left_Back_Leg.GPIOx = Left_Back_Leg_Control_GPIO_Port;
  Left_Back_Leg.GPIO_Pin = Left_Back_Leg_Control_Pin;
  Left_Back_Leg.huartx = &huart7;

  Right_Back_Leg.GPIOx = Right_Back_Leg_Control_GPIO_Port;
  Right_Back_Leg.GPIO_Pin = Right_Back_Leg_Control_Pin;
  Right_Back_Leg.huartx = &huart5;

  for (uint8_t leg = 0; leg < 4; leg++) {
    Legs[leg]->Leg_Status = Leg_Idle;
    HAL_GPIO_WritePin(Legs[leg]->GPIOx, Legs[leg]->GPIO_Pin, GPIO_PIN_RESET);
    for (uint8_t motor = 0; motor < 2; motor++) {
      MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
      Legs[leg]->rotor_zero_offset[motor] = default_rotor_zero_offset[leg][motor];
      Legs[leg]->motor_direction[motor] = default_motor_direction[leg][motor];
      cmd->id = motor;
      cmd->mode = 1;
      cmd->T = 0.0f;
      cmd->W = 0.0f;
      cmd->Pos = 0.0f;
      cmd->K_P = 0.0f;
      cmd->K_W = 0.0f;
      modify_data(cmd);
      Legs[leg]->motor_data[motor].rx_len = 0;
      Legs[leg]->motor_data[motor].correct = 0;
      reset_motor_runtime_state(&Legs[leg]->motor_state[motor],
                                Legs[leg]->rotor_zero_offset[motor],
                                Legs[leg]->motor_direction[motor]);
    }
    Legs[leg]->has_online_motor = Motor_Offline;
  }

  if (Motor_Transport_Init(&motor_transport_callbacks) != HAL_OK) Error_Handler();
}

void Leg_Control_Handshake(void)
{
  if (Motor_Transport_Stop() != HAL_OK) {
    Communication_SendString("MOTOR_TRANSPORT stop_fail\r\n");
    return;
  }

  for (uint8_t leg = 0; leg < 4; leg++)
    leg_abort_transfer(Legs[leg]);

  for (uint8_t leg = 0; leg < 4; leg++) {
    Legs[leg]->has_online_motor = Motor_Offline;
    for (uint8_t motor = 0; motor < 2; motor++) {
      Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
      MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
      MOTOR_recv *fbk = &Legs[leg]->motor_data[motor];
      reset_motor_runtime_state(state,
                                Legs[leg]->rotor_zero_offset[motor],
                                Legs[leg]->motor_direction[motor]);
      cmd->id = motor;
      cmd->mode = 1;
      cmd->T = 0.0f;
      cmd->W = 0;
      cmd->Pos = 0.0f;
      cmd->K_P = 0.0f;
      /* DMA migration validation is communication-only: keep every output zero. */
      cmd->K_W = 0.0f;

      for (uint8_t retry = 0; retry < LEG_HANDSHAKE_RETRY; retry++) {
        HAL_StatusTypeDef ret = SERVO_Send_recv(cmd, fbk, Legs[leg]->GPIOx,
                                                Legs[leg]->GPIO_Pin, Legs[leg]->huartx);
        if (ret == HAL_OK && fbk->correct && fbk->motor_id == motor) {
          Motor_State_UpdateRawFeedback(state, fbk->Pos, fbk->W, fbk->T,
                                        HAL_GetTick(), LEG_REDUCTION_RATIO,
                                        LEG_ROTOR_ZERO_NEAR_RAD);
          state->handshake_status = Motor_Handshake_Ok;
          Legs[leg]->p_init[motor] = fbk->Pos;
          cmd->Pos = fbk->Pos;
          modify_data(cmd);
          break;
        } else if (ret == HAL_TIMEOUT) {
          Motor_State_RecordError(state);
          state->handshake_status = Motor_Handshake_Timeout;
        } else if (ret == HAL_OK) {
          Motor_State_RecordError(state);
          state->handshake_status = Motor_Handshake_BadId;
        } else {
          Motor_State_RecordError(state);
          state->handshake_status = Motor_Handshake_UartError;
        }
      }
    }
    Legs[leg]->has_online_motor = (motor_is_online(leg, 0) || motor_is_online(leg, 1)) ? Motor_Online : Motor_Offline;
    Legs[leg]->Leg_Status = Leg_Idle;
    HAL_GPIO_WritePin(Legs[leg]->GPIOx, Legs[leg]->GPIO_Pin, GPIO_PIN_RESET);
  }

  if (Motor_Transport_Start() != HAL_OK) {
    for (uint8_t leg = 0; leg < 4; ++leg) {
      Legs[leg]->has_online_motor = Motor_Offline;
      for (uint8_t motor = 0; motor < 2; ++motor) {
        Motor_State_RecordError(&Legs[leg]->motor_state[motor]);
        Motor_State_MarkOffline(&Legs[leg]->motor_state[motor]);
        Legs[leg]->motor_state[motor].handshake_status = Motor_Handshake_UartError;
      }
    }
    Communication_SendString("MOTOR_TRANSPORT start_fail\r\n");
  }
}

void Leg_Control_RequestHandshake(void)
{
  handshake_requested = 1;
}

/* ---- motor polling ---- */

static void __attribute__((unused)) poll_online_motor(uint8_t leg, uint8_t motor)
{
  if (leg >= 4 || motor >= 2 || !motor_is_online(leg, motor)) return;

  Leg_HandlerTypeDef *hleg = Legs[leg];
  MOTOR_send *cmd = &hleg->motor_cmd[motor];
  MOTOR_recv *fbk = &hleg->motor_data[motor];
  Motor_RuntimeStateTypeDef *state = &hleg->motor_state[motor];

  hleg->Leg_Status = tx_status_for_motor(motor);
  HAL_StatusTypeDef ret = SERVO_Send_recv(cmd, fbk, hleg->GPIOx, hleg->GPIO_Pin, hleg->huartx);

  for (uint8_t retry = 0; retry < LEG_IO_RETRY_COUNT && !motor_feedback_is_valid(ret, fbk, motor); retry++)
    ret = SERVO_Send_recv(cmd, fbk, hleg->GPIOx, hleg->GPIO_Pin, hleg->huartx);

  if (motor_feedback_is_valid(ret, fbk, motor)) {
    Motor_State_UpdateRawFeedback(state, fbk->Pos, fbk->W, fbk->T,
                                  HAL_GetTick(), LEG_REDUCTION_RATIO,
                                  LEG_ROTOR_ZERO_NEAR_RAD);
    state->io_error_count = 0;
  } else {
    handle_motor_io_failure(leg, motor, ret, fbk);
  }
  hleg->Leg_Status = Leg_Done;
}

void Leg_Control_Start(void)
{
  Motor_Transport_Service();
}

/* ---- main service ---- */

void Leg_Control_Service(uint32_t now_ms)
{
  if (handshake_requested) {
    handshake_requested = 0;
    Leg_Control_Handshake();
  }

  if ((now_ms - last_service_tick) < LEG_SERVICE_PERIOD_MS) return;
  last_service_tick = now_ms;

  /* Communication-only DMA validation: motion and gait services stay disabled. */
}

/* ---- hold / stop ---- */

int Leg_Control_HoldCurrentPosition(void)
{
  Leg_Gait_StopAll();

  for (uint8_t idx = 0; idx < 8; idx++) {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
    if (state == NULL || state->online != Motor_Online || state->angle_valid != Motor_Angle_Valid)
      return 0;
  }

  for (uint8_t leg = 0; leg < 4; leg++) {
    for (uint8_t motor = 0; motor < 2; motor++) {
      Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
      MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];

      state->target_offset = state->angle - Legs[leg]->p_init[motor];
      state->target_active = Motor_Target_Inactive;
      state->target_result = Motor_Target_Done;
      state->target_start_angle = state->angle;
      state->target_start_tick = 0;
      state->target_progress_tick = 0;
      state->target_last_abs_error = 0.0f;
      state->target_stop_error = LEG_WEB_STOP_ERROR_RAD;

      cmd->mode = 1;
      cmd->T = 0.0f;
      cmd->W = 0.0f;
      cmd->Pos = state->angle;
      cmd->K_P = LEG_HOLD_KP;
      cmd->K_W = LEG_HOLD_KW;
      modify_data(cmd);
    }
  }

  Communication_SendString("MOTOR_HOLD_CURRENT ok\r\n");
  return 1;
}

void Leg_Control_StopAllDebugTargets(uint8_t reason)
{
  Motor_TargetResultTypeDef result = (reason == 0U) ? Motor_Target_Stopped : (Motor_TargetResultTypeDef)reason;
  Leg_Gait_StopAll();

  for (uint8_t idx = 0; idx < 8; idx++)
    Leg_Control_StopDebugTarget(idx, result);
}

/* ---- snapshot ---- */

void Leg_Control_LogFootSnapshot(void)
{
  Leg_PointTypeDef foot[4] = {0};
  uint8_t foot_ok[4] = {0};
  uint8_t motor_online[8] = {0};
  uint8_t leg_online[4] = {0};
  uint8_t target_active[8] = {0};
  uint8_t target_result[8] = {0};
  uint8_t zero_ok[8] = {0};
  float zero_error[8] = {0.0f};
  uint8_t all_zero_ok = 0U;

  for (uint8_t leg = 0; leg < 4; leg++)
    foot_ok[leg] = Leg_Control_GetCurrentFoot(leg, &foot[leg]) ? 1U : 0U;
  Leg_Control_GetOnline(motor_online, leg_online);
  Leg_Control_GetTargetStates(target_active, target_result);
  Leg_Control_GetZeroCheck(zero_error, zero_ok, &all_zero_ok);
  (void)zero_error; (void)all_zero_ok; (void)leg_online;

  for (uint8_t leg = 0; leg < 4; leg++)
    if (!motor_online[leg * 2U] || !motor_online[leg * 2U + 1U])
      foot_ok[leg] = 0U;

  char buf[240];
  int len = snprintf(buf, sizeof(buf), "LEG_SNAPSHOT ok=%u%u%u%u x=",
                     foot_ok[0], foot_ok[1], foot_ok[2], foot_ok[3]);
  for (uint8_t leg = 0; leg < 4 && len > 0; leg++) {
    if (leg > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
    len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, foot[leg].x);
  }
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " y=");
  for (uint8_t leg = 0; leg < 4 && len > 0; leg++) {
    if (leg > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
    len = Leg_Control_AppendFixed4(buf, sizeof(buf), len, foot[leg].y);
  }
  if (len > 0) {
    len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                    " o=%u%u%u%u%u%u%u%u z=%u%u%u%u%u%u%u%u a=%u%u%u%u%u%u%u%u r=%u%u%u%u%u%u%u%u\r\n",
                    motor_online[0], motor_online[1], motor_online[2], motor_online[3],
                    motor_online[4], motor_online[5], motor_online[6], motor_online[7],
                    zero_ok[0], zero_ok[1], zero_ok[2], zero_ok[3],
                    zero_ok[4], zero_ok[5], zero_ok[6], zero_ok[7],
                    target_active[0], target_active[1], target_active[2], target_active[3],
                    target_active[4], target_active[5], target_active[6], target_active[7],
                    target_result[0], target_result[1], target_result[2], target_result[3],
                    target_result[4], target_result[5], target_result[6], target_result[7]);
  }
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

/* ---- state queries (ISR-safe) ---- */

void Leg_Control_GetAngles(float angles[8], uint8_t valid[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++) {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(i);
    angles[i] = state->angle;
    valid[i] = state->angle_valid;
  }
  __enable_irq();
}

int Leg_Control_GetMotorStateSnapshot(uint8_t motor_index,
                                      Motor_StateSnapshotTypeDef *state)
{
  if (motor_index >= 8U || state == NULL) return 0;
  __disable_irq();
  Motor_State_GetSnapshot(Leg_Control_MotorState(motor_index), state);
  __enable_irq();
  return 1;
}

void Leg_Control_GetOnline(uint8_t motor_online[8], uint8_t leg_online[4])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++) {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(i);
    motor_online[i] = state->online;
  }
  for (uint8_t i = 0; i < 4; i++)
    leg_online[i] = Legs[i]->has_online_motor;
  __enable_irq();
}

void Leg_Control_GetHandshakeErrors(uint8_t motor_error[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++) {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(i);
    motor_error[i] = state->handshake_status;
  }
  __enable_irq();
}

void Leg_Control_GetTargetStates(uint8_t active[8], uint8_t result[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++) {
    Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(i);
    active[i] = state->target_active;
    result[i] = state->target_result;
  }
  __enable_irq();
}

float Leg_Control_GetZeroThreshold(void)
{
  return LEG_ROTOR_ZERO_NEAR_RAD;
}

void Leg_Control_GetZeroCheck(float zero_error[8], uint8_t zero_ok[8], uint8_t *all_zero_ok)
{
  uint8_t all_ok = 1U;
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t leg = i / 2U;
    uint8_t motor = i % 2U;
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    float error = Motor_State_GetZeroError(state);
    uint8_t ok = state->zero_checked;
    zero_error[i] = error;
    zero_ok[i] = ok;
    if (!ok) all_ok = 0U;
  }
  __enable_irq();
  if (all_zero_ok != NULL) *all_zero_ok = all_ok;
}

int Leg_Control_SetZeroOffsets(uint8_t leg, const float rotor_zero_offset[2], const float motor_direction[2])
{
  if (leg >= 4 || rotor_zero_offset == NULL || motor_direction == NULL) return 0;
  __disable_irq();
  for (uint8_t motor = 0; motor < 2; motor++) {
    Legs[leg]->rotor_zero_offset[motor] = rotor_zero_offset[motor];
    Legs[leg]->motor_direction[motor] = Leg_Control_NormalizedMotorDir(motor_direction[motor]);
    Motor_State_SetCalibration(&Legs[leg]->motor_state[motor],
                               Legs[leg]->rotor_zero_offset[motor],
                               Legs[leg]->motor_direction[motor],
                               LEG_REDUCTION_RATIO,
                               LEG_ROTOR_ZERO_NEAR_RAD);
  }
  __enable_irq();
  return 1;
}

int Leg_Control_SetCurrentPositionAsZero(uint8_t leg)
{
  if (leg >= 4) return 0;
  __disable_irq();
  for (uint8_t motor = 0; motor < 2; motor++) {
    if (Legs[leg]->motor_state[motor].angle_valid != Motor_Angle_Valid) {
      __enable_irq(); return 0;
    }
  }
  for (uint8_t motor = 0; motor < 2; motor++) {
    Legs[leg]->rotor_zero_offset[motor] = Legs[leg]->motor_state[motor].angle;
    Motor_State_SetCalibration(&Legs[leg]->motor_state[motor],
                               Legs[leg]->rotor_zero_offset[motor],
                               Legs[leg]->motor_direction[motor],
                               LEG_REDUCTION_RATIO,
                               LEG_ROTOR_ZERO_NEAR_RAD);
  }
  __enable_irq();
  return 1;
}

int Leg_Control_GetJointAngles(uint8_t leg, Leg_JointAnglesTypeDef *angles, uint8_t theta_valid[2])
{
  if (leg >= 4 || angles == NULL) return 0;

  float joint_angle[2];
  uint8_t angle_valid[2];

  __disable_irq();
  for (uint8_t motor = 0; motor < 2; motor++) {
    joint_angle[motor] = Legs[leg]->motor_state[motor].joint_angle;
    angle_valid[motor] = Legs[leg]->motor_state[motor].angle_valid;
  }
  __enable_irq();

  if (theta_valid != NULL) {
    theta_valid[0] = angle_valid[LEG_MOTOR_THETA1];
    theta_valid[1] = angle_valid[LEG_MOTOR_THETA2];
  }

  angles->theta1 = joint_angle[LEG_MOTOR_THETA1];
  angles->theta2 = joint_angle[LEG_MOTOR_THETA2];
  return 1;
}

int Leg_Control_JointToRotorTargets(uint8_t leg, const Leg_JointAnglesTypeDef *angles, float rotor_targets[2])
{
  if (leg >= 4 || angles == NULL || rotor_targets == NULL) return 0;
  rotor_targets[LEG_MOTOR_THETA1] = joint_to_rotor_angle(leg, LEG_MOTOR_THETA1, angles->theta1);
  rotor_targets[LEG_MOTOR_THETA2] = joint_to_rotor_angle(leg, LEG_MOTOR_THETA2, angles->theta2);
  return 1;
}

/* ---- TX / RX interrupt handlers ---- */

void Leg_Tx_Handler(Leg_HandlerTypeDef *hleg)
{
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_RESET);
  if (hleg->Leg_Status == Leg_TX_M0) {
    hleg->Leg_Status = Leg_RX_M0;
    HAL_UARTEx_ReceiveToIdle_DMA(hleg->huartx,
                                 (uint8_t *)&(hleg->motor_data[0].motor_recv_data),
                                 sizeof(hleg->motor_data[0].motor_recv_data));
  }
  if (hleg->Leg_Status == Leg_TX_M1) {
    hleg->Leg_Status = Leg_RX_M1;
    HAL_UARTEx_ReceiveToIdle_DMA(hleg->huartx,
                                 (uint8_t *)&(hleg->motor_data[1].motor_recv_data),
                                 sizeof(hleg->motor_data[1].motor_recv_data));
  }
}

void Leg_Rx_Handler(Leg_HandlerTypeDef *hleg, uint16_t Size)
{
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_SET);

  if (hleg->Leg_Status == Leg_RX_M0) {
    hleg->motor_data[0].rx_len = Size;
    uint8_t l = leg_index_from_handle(hleg);
    uint8_t idx = l == 0xFF ? 0xFF : Leg_Control_MotorIndex(l, 0);
    if (Size == sizeof(hleg->motor_data[0].motor_recv_data)) {
      extract_data(&hleg->motor_data[0]);
      if (idx < 8) {
        Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
        if (state->online && hleg->motor_data[0].correct)
          Motor_State_UpdateRawFeedback(state, hleg->motor_data[0].Pos,
                                        hleg->motor_data[0].W,
                                        hleg->motor_data[0].T, HAL_GetTick(),
                                        LEG_REDUCTION_RATIO,
                                        LEG_ROTOR_ZERO_NEAR_RAD);
        else {
          state->angle_valid = Motor_Angle_Invalid;
          state->zero_checked = 0U;
        }
      }
    } else if (idx < 8) {
      Leg_Control_MotorState(idx)->angle_valid = Motor_Angle_Invalid;
      Leg_Control_MotorState(idx)->zero_checked = 0U;
    }

    if (l < 4 && motor_is_online(l, 1)) {
      hleg->Leg_Status = Leg_TX_M1;
      modify_data(&(hleg->motor_cmd[1]));
      HAL_UART_Transmit_DMA(hleg->huartx,
                            (uint8_t *)&(hleg->motor_cmd[1].motor_send_data),
                            sizeof(hleg->motor_cmd[1].motor_send_data));
    } else {
      hleg->Leg_Status = Leg_Done;
    }
  }

  if (hleg->Leg_Status == Leg_RX_M1) {
    hleg->motor_data[1].rx_len = Size;
    uint8_t l = leg_index_from_handle(hleg);
    uint8_t idx = l == 0xFF ? 0xFF : Leg_Control_MotorIndex(l, 1);
    if (Size == sizeof(hleg->motor_data[1].motor_recv_data)) {
      extract_data(&hleg->motor_data[1]);
      if (idx < 8) {
        Motor_RuntimeStateTypeDef *state = Leg_Control_MotorState(idx);
        if (state->online && hleg->motor_data[1].correct)
          Motor_State_UpdateRawFeedback(state, hleg->motor_data[1].Pos,
                                        hleg->motor_data[1].W,
                                        hleg->motor_data[1].T, HAL_GetTick(),
                                        LEG_REDUCTION_RATIO,
                                        LEG_ROTOR_ZERO_NEAR_RAD);
        else {
          state->angle_valid = Motor_Angle_Invalid;
          state->zero_checked = 0U;
        }
      }
    } else if (idx < 8) {
      Leg_Control_MotorState(idx)->angle_valid = Motor_Angle_Invalid;
      Leg_Control_MotorState(idx)->zero_checked = 0U;
    }
    hleg->Leg_Status = Leg_Done;
  }
}
