/**
******************************************************************************
  * @file    Leg_Control.c
  * @author  Si-Ghost
  * @brief   Robot leg low-rate diagnostic control.
  ******************************************************************************
  */

#include "Leg_Control.h"
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
#define LEG_WEB_SPEED_KP            0.8f
#define LEG_WEB_SPEED_MIN_RAD_S     0.8f
#define LEG_WEB_SPEED_LIMIT_RAD_S   1.2f
#define LEG_WEB_STOP_ERROR_RAD      0.02f
#define LEG_WEB_TARGET_TIMEOUT_MS   30000U
#define LEG_WEB_NO_PROGRESS_MS      4000U
#define LEG_WEB_STALL_GRACE_MS      500U
#define LEG_WEB_STALL_PROGRESS_RAD  0.005f
#define LEG_FOOT_NUDGE_MAX_MM       15.0f
#define LEG_FOOT_NUDGE_ROTOR_RAD    0.65f
#define LEG_WEB_KP                  0.0f
#define LEG_WEB_KW                  0.05f
#define LEG_HANDSHAKE_KW            0.05f
#define LEG_HANDSHAKE_RETRY         2U
#define LEG_DEBUG_LOG_PERIOD_MS     250U
#define LEG_IO_ERROR_LOG_PERIOD_MS  1000U
#define LEG_IO_ERROR_OFFLINE_COUNT  5U
#define LEG_IO_RETRY_COUNT          1U
#define LEG_LINK_L1_MM              130.0f
#define LEG_LINK_L2_MM              260.0f
#define LEG_REDUCTION_RATIO         6.33f
#define LEG_PI                      3.14159265358979323846f
#define LEG_TWO_PI                  6.28318530717958647692f
#define LEG_KIN_EPSILON             0.000001f
#define LEG_ROTOR_ZERO_NEAR_RAD     1.50f
#define LEG_MOTOR_THETA1            1U
#define LEG_MOTOR_THETA2            0U

static uint32_t last_service_tick = 0;
static volatile uint8_t handshake_requested = 0;

static void leg_abort_transfer(Leg_HandlerTypeDef *hleg);

static const float default_rotor_zero_offset[4][2] = {
  {4.2988f, 3.4371f},  // LF: motor0(theta2), motor1(theta1)
  {3.4235f, 5.9666f},  // RF: motor0(theta2), motor1(theta1)
  {5.9979f, 1.0314f},  // LB: motor0(theta2), motor1(theta1)
  {1.4003f, 5.2892f},  // RB: motor0(theta2), motor1(theta1)
};

static const float default_motor_direction[4][2] = {
  {1.0f, 1.0f},
  {1.0f, 1.0f},
  {1.0f, 1.0f},
  {1.0f, 1.0f},
};

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static float absf_local(float value)
{
  return value < 0.0f ? -value : value;
}

static float rotor_wrap_delta(float angle, float reference)
{
  float delta = angle - reference;
  while (delta > LEG_PI)
    delta -= LEG_TWO_PI;
  while (delta < -LEG_PI)
    delta += LEG_TWO_PI;
  return delta;
}

static uint8_t leg_index_from_handle(Leg_HandlerTypeDef *hleg)
{
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    if (Legs[leg] == hleg)
      return leg;
  }
  return 0xFF;
}

static uint8_t motor_index_from_leg_motor(uint8_t leg, uint8_t motor)
{
  return leg * 2U + motor;
}

static Motor_RuntimeStateTypeDef *motor_state_from_leg_motor(uint8_t leg, uint8_t motor)
{
  if (leg >= 4 || motor >= 2)
    return NULL;

  return &Legs[leg]->motor_state[motor];
}

static Motor_RuntimeStateTypeDef *motor_state_from_index(uint8_t motor_index)
{
  if (motor_index >= 8)
    return NULL;

  return motor_state_from_leg_motor(motor_index / 2U, motor_index % 2U);
}

static float normalized_motor_direction(float direction)
{
  return (direction < 0.0f) ? -1.0f : 1.0f;
}

static float joint_to_rotor_angle(const Leg_HandlerTypeDef *hleg, uint8_t motor, float joint_angle)
{
  float direction = normalized_motor_direction(hleg->motor_direction[motor]);
  return hleg->rotor_zero_offset[motor] + direction * joint_angle * LEG_REDUCTION_RATIO;
}

static void reset_motor_runtime_state(Motor_RuntimeStateTypeDef *state)
{
  if (state == NULL)
    return;

  state->angle = 0.0f;
  state->angle_valid = Motor_Angle_Invalid;
  state->online = Motor_Offline;
  state->handshake_status = Motor_Handshake_Timeout;
  state->target_offset = 0.0f;
  state->target_active = Motor_Target_Inactive;
  state->target_result = Motor_Target_Idle;
  state->target_start_tick = 0;
  state->target_progress_tick = 0;
  state->target_last_abs_error = 0.0f;
  state->debug_last_log_tick = 0;
  state->io_error_last_log_tick = 0;
  state->io_error_count = 0;
}

static uint8_t motor_is_online(uint8_t leg, uint8_t motor)
{
  Motor_RuntimeStateTypeDef *state = motor_state_from_leg_motor(leg, motor);
  if (state == NULL)
    return 0;

  return state->online != 0;
}

static Leg_StatusTypeDef tx_status_for_motor(uint8_t motor)
{
  return (motor == 0) ? Leg_TX_M0 : Leg_TX_M1;
}

static int append_fixed4(char *buf, size_t size, int pos, float value)
{
  if (pos < 0 || (size_t)pos >= size)
    return -1;

  int32_t scaled;
  if (value >= 0.0f)
    scaled = (int32_t)(value * 10000.0f + 0.5f);
  else
    scaled = (int32_t)(value * 10000.0f - 0.5f);

  const char *sign = "";
  if (scaled < 0)
  {
    sign = "-";
    scaled = -scaled;
  }

  int written = snprintf(&buf[pos], size - (size_t)pos, "%s%ld.%04ld",
                         sign,
                         (long)(scaled / 10000),
                         (long)(scaled % 10000));
  if (written < 0 || written >= (int)(size - (size_t)pos))
    return -1;

  return pos + written;
}

static void log_debug_target(uint8_t motor_index, const char *tag, float desired, float delta)
{
  if (motor_index >= 8)
    return;

  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
  Motor_RuntimeStateTypeDef *state = motor_state_from_index(motor_index);

  char buf[192];
  int len = snprintf(buf, sizeof(buf), "MOTOR_CMD %s idx=%u off=", tag, motor_index);
  len = append_fixed4(buf, sizeof(buf), len, state->target_offset);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " cmd=");
  len = append_fixed4(buf, sizeof(buf), len, cmd->Pos);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " spd=");
  len = append_fixed4(buf, sizeof(buf), len, cmd->W);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " target=");
  len = append_fixed4(buf, sizeof(buf), len, desired);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " fbk=");
  len = append_fixed4(buf, sizeof(buf), len, state->angle);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " delta=");
  len = append_fixed4(buf, sizeof(buf), len, delta);
  if (len > 0)
  {
    int written = snprintf(&buf[len], sizeof(buf) - (size_t)len,
                           " raw_pos=%ld raw_spd=%d raw_kp=%d raw_kw=%d active=%u\r\n",
                           (long)cmd->motor_send_data.comd.pos_des,
                           (int)cmd->motor_send_data.comd.spd_des,
                           (int)cmd->motor_send_data.comd.k_pos,
                           (int)cmd->motor_send_data.comd.k_spd,
                           state->target_active);
    if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
      return;
    len += written;
  }

  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void stop_debug_target(uint8_t motor_index, Motor_TargetResultTypeDef result)
{
  if (motor_index >= 8)
    return;

  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
  Motor_RuntimeStateTypeDef *state = motor_state_from_index(motor_index);

  state->target_active = Motor_Target_Inactive;
  state->target_result = result;
  state->target_start_tick = 0;
  state->target_progress_tick = 0;
  state->target_last_abs_error = 0.0f;

  cmd->mode = 1;
  cmd->T = 0.0f;
  cmd->W = 0.0f;
  cmd->K_P = 0.0f;
  cmd->K_W = 0.0f;
  cmd->Pos = state->angle;
  modify_data(cmd);
}

static void log_motor_io_error(uint8_t motor_index, HAL_StatusTypeDef ret, MOTOR_recv *fbk)
{
  if (motor_index >= 8)
    return;

  uint32_t now = HAL_GetTick();
  Motor_RuntimeStateTypeDef *state = motor_state_from_index(motor_index);
  if ((now - state->io_error_last_log_tick) < LEG_IO_ERROR_LOG_PERIOD_MS)
    return;

  state->io_error_last_log_tick = now;

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "MOTOR_IO fail idx=%u ret=%d rxlen=%u ok=%d err=%u cnt=%u h=%02X%02X id=%u\r\n",
                     motor_index,
                     (int)ret,
                     (unsigned int)fbk->rx_len,
                     fbk->correct,
                     fbk->MError,
                     state->io_error_count,
                     fbk->motor_recv_data.head[0],
                     fbk->motor_recv_data.head[1],
                     fbk->motor_recv_data.mode.id);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static void refresh_leg_online_state(uint8_t leg)
{
  if (leg >= 4)
    return;

  Legs[leg]->has_online_motor =
      (motor_is_online(leg, 0) || motor_is_online(leg, 1)) ? Motor_Online : Motor_Offline;
}

static void handle_motor_io_failure(uint8_t leg, uint8_t motor, HAL_StatusTypeDef ret, MOTOR_recv *fbk)
{
  if (leg >= 4 || motor >= 2)
    return;

  uint8_t idx = motor_index_from_leg_motor(leg, motor);
  Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];

  if (state->io_error_count < 255U)
    state->io_error_count++;

  state->angle_valid = Motor_Angle_Invalid;
  log_motor_io_error(idx, ret, fbk);

  if (state->io_error_count < LEG_IO_ERROR_OFFLINE_COUNT)
    return;

  stop_debug_target(idx, Motor_Target_Stopped);
  state->online = Motor_Offline;
  state->handshake_status = (ret == HAL_TIMEOUT) ? Motor_Handshake_Timeout : Motor_Handshake_UartError;
  refresh_leg_online_state(leg);
  leg_abort_transfer(Legs[leg]);

  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "MOTOR_IO offline idx=%u ret=%d rxlen=%u cnt=%u h=%02X%02X id=%u rescan required\r\n",
                     idx,
                     (int)ret,
                     (unsigned int)fbk->rx_len,
                     state->io_error_count,
                     fbk->motor_recv_data.head[0],
                     fbk->motor_recv_data.head[1],
                     fbk->motor_recv_data.mode.id);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static uint8_t motor_feedback_is_valid(HAL_StatusTypeDef ret, MOTOR_recv *fbk, uint8_t motor)
{
  return (ret == HAL_OK && fbk->correct && fbk->motor_id == motor) ? 1U : 0U;
}

static int apply_debug_target(uint8_t motor_index, uint8_t force_log)
{
  Motor_RuntimeStateTypeDef *state = motor_state_from_index(motor_index);
  if (state == NULL || !state->online)
    return 0;

  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];

  float desired = Legs[leg]->p_init[motor] + state->target_offset;
  float error = desired - state->angle;
  float abs_error = absf_local(error);
  uint32_t now = HAL_GetTick();

  if (state->target_active)
  {
    if (abs_error <= LEG_WEB_STOP_ERROR_RAD)
    {
      stop_debug_target(motor_index, Motor_Target_Done);
    }
    else if ((now - state->target_start_tick) >= LEG_WEB_TARGET_TIMEOUT_MS)
    {
      stop_debug_target(motor_index, Motor_Target_Timeout);
    }
    else if ((state->target_last_abs_error - abs_error) >= LEG_WEB_STALL_PROGRESS_RAD)
    {
      state->target_last_abs_error = abs_error;
      state->target_progress_tick = now;
    }
    else if ((now - state->target_start_tick) >= LEG_WEB_STALL_GRACE_MS &&
             (now - state->target_progress_tick) >= LEG_WEB_NO_PROGRESS_MS)
    {
      stop_debug_target(motor_index, Motor_Target_Stall);
    }
  }

  cmd->mode = 1;
  cmd->T = 0.0f;
  if (state->target_active)
  {
    float speed = clampf(error * LEG_WEB_SPEED_KP,
                         -LEG_WEB_SPEED_LIMIT_RAD_S,
                         LEG_WEB_SPEED_LIMIT_RAD_S);
    if (absf_local(speed) < LEG_WEB_SPEED_MIN_RAD_S)
      speed = (error >= 0.0f) ? LEG_WEB_SPEED_MIN_RAD_S : -LEG_WEB_SPEED_MIN_RAD_S;
    cmd->W = speed;
  }
  else
  {
    cmd->W = 0.0f;
  }
  cmd->Pos = state->target_active ? desired : state->angle;
  cmd->K_P = state->target_active ? LEG_WEB_KP : 0.0f;
  cmd->K_W = state->target_active ? LEG_WEB_KW : 0.0f;
  modify_data(cmd);

  if (force_log || !state->target_active ||
      (now - state->debug_last_log_tick) >= LEG_DEBUG_LOG_PERIOD_MS)
  {
    state->debug_last_log_tick = now;
    log_debug_target(motor_index,
                     state->target_active ? "step" : "done",
                     desired,
                     error);
  }

  return 1;
}

static void update_debug_targets(void)
{
  for (uint8_t idx = 0; idx < 8; idx++)
  {
    Motor_RuntimeStateTypeDef *state = motor_state_from_index(idx);
    if (state != NULL && state->target_active)
      apply_debug_target(idx, 0);
  }
}

static void leg_abort_transfer(Leg_HandlerTypeDef *hleg)
{
  if (hleg == NULL)
    return;

  if (hleg->huartx != NULL)
    HAL_UART_Abort(hleg->huartx);

  hleg->Leg_Status = Leg_Idle;
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_RESET);
}

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

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    Legs[leg]->Leg_Status = Leg_Idle;
    HAL_GPIO_WritePin(Legs[leg]->GPIOx, Legs[leg]->GPIO_Pin, GPIO_PIN_RESET);

    for (uint8_t motor = 0; motor < 2; motor++)
    {
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
      reset_motor_runtime_state(&Legs[leg]->motor_state[motor]);
    }
    Legs[leg]->has_online_motor = Motor_Offline;
  }
}

void Leg_Control_Handshake(void)
{
  for (uint8_t leg = 0; leg < 4; leg++)
  {
    leg_abort_transfer(Legs[leg]);
  }

  for (uint8_t leg = 0; leg < 4; leg++)
  {
    Legs[leg]->has_online_motor = Motor_Offline;

    for (uint8_t motor = 0; motor < 2; motor++)
    {
      Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
      MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
      MOTOR_recv *fbk = &Legs[leg]->motor_data[motor];

      reset_motor_runtime_state(state);
      cmd->id = motor;
      cmd->mode = 1;
      cmd->T = 0.0f;
      cmd->W = 0;
      cmd->Pos = 0.0f;
      cmd->K_P = 0.0f;
      cmd->K_W = LEG_HANDSHAKE_KW;

      for (uint8_t retry = 0; retry < LEG_HANDSHAKE_RETRY; retry++)
      {
        HAL_StatusTypeDef ret = SERVO_Send_recv(cmd, fbk, Legs[leg]->GPIOx,
                                                Legs[leg]->GPIO_Pin,
                                                Legs[leg]->huartx);
        if (ret == HAL_OK && fbk->correct && fbk->motor_id == motor)
        {
          state->online = Motor_Online;
          state->angle_valid = Motor_Angle_Valid;
          state->handshake_status = Motor_Handshake_Ok;
          state->angle = fbk->Pos;
          Legs[leg]->p_init[motor] = fbk->Pos;
          cmd->Pos = fbk->Pos;
          modify_data(cmd);
          break;
        }
        else if (ret == HAL_TIMEOUT)
        {
          state->handshake_status = Motor_Handshake_Timeout;
        }
        else if (ret == HAL_OK)
        {
          state->handshake_status = Motor_Handshake_BadId;
        }
        else
        {
          state->handshake_status = Motor_Handshake_UartError;
        }
      }
    }

    Legs[leg]->has_online_motor = (motor_is_online(leg, 0) || motor_is_online(leg, 1)) ? Motor_Online : Motor_Offline;
    Legs[leg]->Leg_Status = Leg_Idle;
    HAL_GPIO_WritePin(Legs[leg]->GPIOx, Legs[leg]->GPIO_Pin, GPIO_PIN_RESET);
  }
}

void Leg_Control_RequestHandshake(void)
{
  handshake_requested = 1;
}

static void poll_online_motor(uint8_t leg, uint8_t motor)
{
  if (leg >= 4 || motor >= 2 || !motor_is_online(leg, motor))
    return;

  Leg_HandlerTypeDef *hleg = Legs[leg];
  MOTOR_send *cmd = &hleg->motor_cmd[motor];
  MOTOR_recv *fbk = &hleg->motor_data[motor];
  Motor_RuntimeStateTypeDef *state = &hleg->motor_state[motor];

  hleg->Leg_Status = tx_status_for_motor(motor);
  HAL_StatusTypeDef ret = SERVO_Send_recv(cmd, fbk,
                                          hleg->GPIOx,
                                          hleg->GPIO_Pin,
                                          hleg->huartx);

  for (uint8_t retry = 0; retry < LEG_IO_RETRY_COUNT && !motor_feedback_is_valid(ret, fbk, motor); retry++)
  {
    ret = SERVO_Send_recv(cmd, fbk,
                          hleg->GPIOx,
                          hleg->GPIO_Pin,
                          hleg->huartx);
  }

  if (motor_feedback_is_valid(ret, fbk, motor))
  {
    state->angle = fbk->Pos;
    state->angle_valid = Motor_Angle_Valid;
    state->io_error_count = 0;
  }
  else
  {
    handle_motor_io_failure(leg, motor, ret, fbk);
  }

  hleg->Leg_Status = Leg_Done;
}

void Leg_Control_Start(void)
{
  for (int i = 0; i < 4; i++)
  {
    if (!Legs[i]->has_online_motor)
      continue;

    poll_online_motor((uint8_t)i, 0);
    poll_online_motor((uint8_t)i, 1);
  }
}

void Leg_Control_Service(uint32_t now_ms)
{
  if (handshake_requested)
  {
    handshake_requested = 0;
    Leg_Control_Handshake();
  }

  if ((now_ms - last_service_tick) < LEG_SERVICE_PERIOD_MS)
    return;

  last_service_tick = now_ms;
  update_debug_targets();
  Leg_Control_Start();
}

int Leg_Control_SetDebugAngle(uint8_t motor_index, float angle_rad)
{
  Motor_RuntimeStateTypeDef *state = motor_state_from_index(motor_index);
  if (state == NULL)
    return 0;

  if (!state->online)
    return 0;

  state->target_offset = clampf(angle_rad, LEG_WEB_ANGLE_MIN_RAD, LEG_WEB_ANGLE_MAX_RAD);
  state->target_active = Motor_Target_Active;
  state->target_result = Motor_Target_Running;
  state->target_start_tick = HAL_GetTick();
  state->target_progress_tick = state->target_start_tick;
  state->target_last_abs_error =
      absf_local((Legs[motor_index / 2]->p_init[motor_index % 2] + state->target_offset) -
                 state->angle);
  return apply_debug_target(motor_index, 1);
}

static void start_debug_offset(uint8_t motor_index, float offset)
{
  Motor_RuntimeStateTypeDef *state = motor_state_from_index(motor_index);
  if (state == NULL)
    return;

  state->target_offset = offset;
  state->target_active = Motor_Target_Active;
  state->target_result = Motor_Target_Running;
  state->target_start_tick = HAL_GetTick();
  state->target_progress_tick = state->target_start_tick;
  state->target_last_abs_error =
      absf_local((Legs[motor_index / 2]->p_init[motor_index % 2] + state->target_offset) -
                 state->angle);
}

static void log_leg_nudge_reject(uint8_t leg, const char *reason, uint8_t motor, float value)
{
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "LEG_NUDGE reject leg=%u reason=%s motor=%u value=",
                     leg,
                     reason,
                     motor);
  len = append_fixed4(buf, sizeof(buf), len, value);
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

int Leg_Control_SetDebugFootOffset(uint8_t leg, float dx_mm, float dy_mm)
{
  if (leg >= 4)
    return 0;

  dx_mm = clampf(dx_mm, -LEG_FOOT_NUDGE_MAX_MM, LEG_FOOT_NUDGE_MAX_MM);
  dy_mm = clampf(dy_mm, -LEG_FOOT_NUDGE_MAX_MM, LEG_FOOT_NUDGE_MAX_MM);

  for (uint8_t motor = 0; motor < 2; motor++)
  {
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    if (state->online != Motor_Online)
    {
      log_leg_nudge_reject(leg, "offline", motor, 0.0f);
      return 0;
    }
    if (state->angle_valid != Motor_Angle_Valid)
    {
      log_leg_nudge_reject(leg, "angle_invalid", motor, 0.0f);
      return 0;
    }

    float home_zero_error = rotor_wrap_delta(Legs[leg]->p_init[motor],
                                             Legs[leg]->rotor_zero_offset[motor]);
    if (absf_local(home_zero_error) > LEG_ROTOR_ZERO_NEAR_RAD)
    {
      log_leg_nudge_reject(leg, "home_zero_out", motor, home_zero_error);
      return 0;
    }
  }

  Leg_JointAnglesTypeDef current_angles;
  uint8_t theta_valid[2] = {0, 0};
  if (!Leg_Control_GetJointAngles(leg, &current_angles, theta_valid) ||
      !theta_valid[0] || !theta_valid[1])
  {
    log_leg_nudge_reject(leg, "theta_invalid", 0U, 0.0f);
    return 0;
  }

  Leg_PointTypeDef current_foot;
  if (!Leg_Kinematics_Forward(&current_angles, &current_foot))
  {
    log_leg_nudge_reject(leg, "fk_fail", 0U, 0.0f);
    return 0;
  }

  Leg_PointTypeDef target_foot = {
      .x = current_foot.x + dx_mm,
      .y = current_foot.y + dy_mm,
  };
  Leg_JointAnglesTypeDef target_angles;
  if (!Leg_Kinematics_Inverse(&target_foot, &target_angles))
  {
    log_leg_nudge_reject(leg, "ik_fail", 0U, target_foot.y);
    return 0;
  }

  float rotor_targets[2];
  if (!Leg_Control_JointToRotorTargets(leg, &target_angles, rotor_targets))
  {
    log_leg_nudge_reject(leg, "rotor_target_fail", 0U, 0.0f);
    return 0;
  }

  float offsets[2];
  for (uint8_t motor = 0; motor < 2; motor++)
  {
    offsets[motor] = rotor_wrap_delta(rotor_targets[motor], Legs[leg]->p_init[motor]);
    if (absf_local(offsets[motor]) > LEG_FOOT_NUDGE_ROTOR_RAD)
    {
      log_leg_nudge_reject(leg, "rotor_offset_limit", motor, offsets[motor]);
      return 0;
    }
  }

  start_debug_offset(motor_index_from_leg_motor(leg, 0), offsets[0]);
  start_debug_offset(motor_index_from_leg_motor(leg, 1), offsets[1]);
  int ok0 = apply_debug_target(motor_index_from_leg_motor(leg, 0), 1);
  int ok1 = apply_debug_target(motor_index_from_leg_motor(leg, 1), 1);

  char buf[192];
  int len = snprintf(buf, sizeof(buf), "LEG_NUDGE ok leg=%u foot=", leg);
  len = append_fixed4(buf, sizeof(buf), len, current_foot.x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = append_fixed4(buf, sizeof(buf), len, current_foot.y);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " target=");
  len = append_fixed4(buf, sizeof(buf), len, target_foot.x);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = append_fixed4(buf, sizeof(buf), len, target_foot.y);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " rotor_off=");
  len = append_fixed4(buf, sizeof(buf), len, offsets[0]);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, ",");
  len = append_fixed4(buf, sizeof(buf), len, offsets[1]);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, "\r\n");
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);

  return ok0 && ok1;
}

void Leg_Control_StopAllDebugTargets(uint8_t reason)
{
  Motor_TargetResultTypeDef result = (reason == 0U) ? Motor_Target_Stopped : (Motor_TargetResultTypeDef)reason;

  for (uint8_t idx = 0; idx < 8; idx++)
  {
    stop_debug_target(idx, result);
  }
}

void Leg_Control_GetAngles(float angles[8], uint8_t valid[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
  {
    Motor_RuntimeStateTypeDef *state = motor_state_from_index(i);
    angles[i] = state->angle;
    valid[i] = state->angle_valid;
  }
  __enable_irq();
}

void Leg_Control_GetOnline(uint8_t motor_online[8], uint8_t leg_online[4])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
  {
    Motor_RuntimeStateTypeDef *state = motor_state_from_index(i);
    motor_online[i] = state->online;
  }
  for (uint8_t i = 0; i < 4; i++)
    leg_online[i] = Legs[i]->has_online_motor;
  __enable_irq();
}

void Leg_Control_GetHandshakeErrors(uint8_t motor_error[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
  {
    Motor_RuntimeStateTypeDef *state = motor_state_from_index(i);
    motor_error[i] = state->handshake_status;
  }
  __enable_irq();
}

void Leg_Control_GetTargetStates(uint8_t active[8], uint8_t result[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
  {
    Motor_RuntimeStateTypeDef *state = motor_state_from_index(i);
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
  for (uint8_t i = 0; i < 8; i++)
  {
    uint8_t leg = i / 2U;
    uint8_t motor = i % 2U;
    Motor_RuntimeStateTypeDef *state = &Legs[leg]->motor_state[motor];
    float error = rotor_wrap_delta(state->angle, Legs[leg]->rotor_zero_offset[motor]);
    uint8_t ok = (state->online == Motor_Online &&
                  state->angle_valid == Motor_Angle_Valid &&
                  absf_local(error) <= LEG_ROTOR_ZERO_NEAR_RAD) ? 1U : 0U;

    zero_error[i] = error;
    zero_ok[i] = ok;
    if (!ok)
      all_ok = 0U;
  }
  __enable_irq();

  if (all_zero_ok != NULL)
    *all_zero_ok = all_ok;
}

int Leg_Control_SetZeroOffsets(uint8_t leg, const float rotor_zero_offset[2], const float motor_direction[2])
{
  if (leg >= 4 || rotor_zero_offset == NULL || motor_direction == NULL)
    return 0;

  __disable_irq();
  for (uint8_t motor = 0; motor < 2; motor++)
  {
    Legs[leg]->rotor_zero_offset[motor] = rotor_zero_offset[motor];
    Legs[leg]->motor_direction[motor] = normalized_motor_direction(motor_direction[motor]);
  }
  __enable_irq();
  return 1;
}

int Leg_Control_SetCurrentPositionAsZero(uint8_t leg)
{
  if (leg >= 4)
    return 0;

  __disable_irq();
  for (uint8_t motor = 0; motor < 2; motor++)
  {
    if (Legs[leg]->motor_state[motor].angle_valid != Motor_Angle_Valid)
    {
      __enable_irq();
      return 0;
    }
  }

  for (uint8_t motor = 0; motor < 2; motor++)
  {
    Legs[leg]->rotor_zero_offset[motor] = Legs[leg]->motor_state[motor].angle;
  }
  __enable_irq();
  return 1;
}

int Leg_Control_GetJointAngles(uint8_t leg, Leg_JointAnglesTypeDef *angles, uint8_t theta_valid[2])
{
  if (leg >= 4 || angles == NULL)
    return 0;

  float rotor_angle[2];
  float rotor_zero_offset[2];
  float motor_direction[2];
  uint8_t angle_valid[2];

  __disable_irq();
  for (uint8_t motor = 0; motor < 2; motor++)
  {
    rotor_angle[motor] = Legs[leg]->motor_state[motor].angle;
    rotor_zero_offset[motor] = Legs[leg]->rotor_zero_offset[motor];
    motor_direction[motor] = Legs[leg]->motor_direction[motor];
    angle_valid[motor] = Legs[leg]->motor_state[motor].angle_valid;
  }
  __enable_irq();

  if (theta_valid != NULL)
  {
    theta_valid[0] = angle_valid[LEG_MOTOR_THETA1];
    theta_valid[1] = angle_valid[LEG_MOTOR_THETA2];
  }

  angles->theta1 = normalized_motor_direction(motor_direction[LEG_MOTOR_THETA1]) *
                   rotor_wrap_delta(rotor_angle[LEG_MOTOR_THETA1], rotor_zero_offset[LEG_MOTOR_THETA1]) /
                   LEG_REDUCTION_RATIO;
  angles->theta2 = normalized_motor_direction(motor_direction[LEG_MOTOR_THETA2]) *
                   rotor_wrap_delta(rotor_angle[LEG_MOTOR_THETA2], rotor_zero_offset[LEG_MOTOR_THETA2]) /
                   LEG_REDUCTION_RATIO;
  return 1;
}

int Leg_Control_JointToRotorTargets(uint8_t leg, const Leg_JointAnglesTypeDef *angles, float rotor_targets[2])
{
  if (leg >= 4 || angles == NULL || rotor_targets == NULL)
    return 0;

  rotor_targets[LEG_MOTOR_THETA1] = joint_to_rotor_angle(Legs[leg], LEG_MOTOR_THETA1, angles->theta1);
  rotor_targets[LEG_MOTOR_THETA2] = joint_to_rotor_angle(Legs[leg], LEG_MOTOR_THETA2, angles->theta2);
  return 1;
}

int Leg_Kinematics_Forward(const Leg_JointAnglesTypeDef *angles, Leg_PointTypeDef *foot)
{
  if (angles == NULL || foot == NULL)
    return 0;

  float theta1 = angles->theta1;
  float theta2 = angles->theta2;
  float cos1 = cosf(theta1);
  float cos2 = cosf(theta2);
  float sin1 = sinf(theta1);
  float sin2 = sinf(theta2);
  float cos_sum = cosf(theta1 + theta2);
  float d_sq = LEG_LINK_L1_MM * LEG_LINK_L1_MM * (2.0f + 2.0f * cos_sum);
  if (d_sq <= LEG_KIN_EPSILON)
    return 0;

  float h_sq = LEG_LINK_L2_MM * LEG_LINK_L2_MM - 0.25f * d_sq;
  if (h_sq < -LEG_KIN_EPSILON)
    return 0;
  if (h_sq < 0.0f)
    h_sq = 0.0f;

  float d = sqrtf(d_sq);
  float h = sqrtf(h_sq);
  float x_e = 0.5f * LEG_LINK_L1_MM * (cos2 - cos1);
  float y_e = 0.5f * LEG_LINK_L1_MM * (sin1 + sin2);
  float h_over_d = h / d;

  foot->x = x_e + h_over_d * LEG_LINK_L1_MM * (sin1 - sin2);
  foot->y = y_e + h_over_d * LEG_LINK_L1_MM * (cos1 + cos2);
  return 1;
}

int Leg_Kinematics_Inverse(const Leg_PointTypeDef *foot, Leg_JointAnglesTypeDef *angles)
{
  if (foot == NULL || angles == NULL)
    return 0;

  float x = foot->x;
  float y = foot->y;
  float r_sq = x * x + y * y;
  if (r_sq <= LEG_KIN_EPSILON)
    return 0;

  float r = sqrtf(r_sq);
  float max_reach = LEG_LINK_L1_MM + LEG_LINK_L2_MM;
  float min_reach = absf_local(LEG_LINK_L2_MM - LEG_LINK_L1_MM);
  if (r > max_reach || r < min_reach)
    return 0;

  float cos_alpha = (LEG_LINK_L1_MM * LEG_LINK_L1_MM + r_sq - LEG_LINK_L2_MM * LEG_LINK_L2_MM) /
                    (2.0f * LEG_LINK_L1_MM * r);
  if (cos_alpha > 1.0f)
    cos_alpha = 1.0f;
  else if (cos_alpha < -1.0f)
    cos_alpha = -1.0f;

  float phi = atan2f(y, x);
  float alpha = acosf(cos_alpha);
  angles->theta2 = phi - alpha;
  angles->theta1 = LEG_PI - alpha - phi;
  return 1;
}

void Leg_Tx_Handler(Leg_HandlerTypeDef *hleg)
{
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_RESET);

  if (hleg->Leg_Status == Leg_TX_M0)
  {
    hleg->Leg_Status = Leg_RX_M0;
    HAL_UARTEx_ReceiveToIdle_DMA(hleg->huartx,
                                 (uint8_t *)&(hleg->motor_data[0].motor_recv_data),
                                 sizeof(hleg->motor_data[0].motor_recv_data));
  }

  if (hleg->Leg_Status == Leg_TX_M1)
  {
    hleg->Leg_Status = Leg_RX_M1;
    HAL_UARTEx_ReceiveToIdle_DMA(hleg->huartx,
                                 (uint8_t *)&(hleg->motor_data[1].motor_recv_data),
                                 sizeof(hleg->motor_data[1].motor_recv_data));
  }
}

void Leg_Rx_Handler(Leg_HandlerTypeDef *hleg, uint16_t Size)
{
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_SET);

  if (hleg->Leg_Status == Leg_RX_M0)
  {
    hleg->motor_data[0].rx_len = Size;
    uint8_t leg = leg_index_from_handle(hleg);
    uint8_t idx = leg == 0xFF ? 0xFF : motor_index_from_leg_motor(leg, 0);
    if (Size == sizeof(hleg->motor_data[0].motor_recv_data))
    {
      extract_data(&hleg->motor_data[0]);
      if (idx < 8)
      {
        Motor_RuntimeStateTypeDef *state = motor_state_from_index(idx);
        state->angle = hleg->motor_data[0].Pos;
        state->angle_valid = (state->online && hleg->motor_data[0].correct) ? Motor_Angle_Valid : Motor_Angle_Invalid;
      }
    }
    else if (idx < 8)
    {
      Motor_RuntimeStateTypeDef *state = motor_state_from_index(idx);
      state->angle_valid = Motor_Angle_Invalid;
    }

    if (leg < 4 && motor_is_online(leg, 1))
    {
      hleg->Leg_Status = Leg_TX_M1;
      modify_data(&(hleg->motor_cmd[1]));
      HAL_UART_Transmit_DMA(hleg->huartx,
                            (uint8_t *)&(hleg->motor_cmd[1].motor_send_data),
                            sizeof(hleg->motor_cmd[1].motor_send_data));
    }
    else
    {
      hleg->Leg_Status = Leg_Done;
    }
  }

  if (hleg->Leg_Status == Leg_RX_M1)
  {
    hleg->motor_data[1].rx_len = Size;
    uint8_t leg = leg_index_from_handle(hleg);
    uint8_t idx = leg == 0xFF ? 0xFF : motor_index_from_leg_motor(leg, 1);
    if (Size == sizeof(hleg->motor_data[1].motor_recv_data))
    {
      extract_data(&hleg->motor_data[1]);
      if (idx < 8)
      {
        Motor_RuntimeStateTypeDef *state = motor_state_from_index(idx);
        state->angle = hleg->motor_data[1].Pos;
        state->angle_valid = (state->online && hleg->motor_data[1].correct) ? Motor_Angle_Valid : Motor_Angle_Invalid;
      }
    }
    else if (idx < 8)
    {
      Motor_RuntimeStateTypeDef *state = motor_state_from_index(idx);
      state->angle_valid = Motor_Angle_Invalid;
    }

    hleg->Leg_Status = Leg_Done;
  }
}
