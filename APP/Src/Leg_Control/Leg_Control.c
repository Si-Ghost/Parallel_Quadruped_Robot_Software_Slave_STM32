/**
******************************************************************************
  * @file    Leg_Control.c
  * @author  Si-Ghost
  * @brief   Robot leg low-rate diagnostic control.
  ******************************************************************************
  */

#include "Leg_Control.h"
#include "communication.h"
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
#define LEG_WEB_KP                  0.0f
#define LEG_WEB_KW                  0.05f
#define LEG_HANDSHAKE_KW            0.05f
#define LEG_HANDSHAKE_RETRY         2U
#define LEG_DEBUG_LOG_PERIOD_MS     250U
#define LEG_IO_ERROR_LOG_PERIOD_MS  1000U

static uint32_t last_service_tick = 0;
static volatile uint8_t handshake_requested = 0;

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

  char buf[96];
  int len = snprintf(buf, sizeof(buf),
                     "MOTOR_IO fail idx=%u ret=%d rxlen=%u ok=%d err=%u\r\n",
                     motor_index,
                     (int)ret,
                     (unsigned int)fbk->rx_len,
                     fbk->correct,
                     fbk->MError);
  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
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
    Legs[leg]->online = Motor_Offline;
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
    Legs[leg]->online = Motor_Offline;

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

    Legs[leg]->online = (motor_is_online(leg, 0) || motor_is_online(leg, 1)) ? Motor_Online : Motor_Offline;
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

  uint8_t idx = motor_index_from_leg_motor(leg, motor);
  Leg_HandlerTypeDef *hleg = Legs[leg];
  MOTOR_send *cmd = &hleg->motor_cmd[motor];
  MOTOR_recv *fbk = &hleg->motor_data[motor];
  Motor_RuntimeStateTypeDef *state = &hleg->motor_state[motor];

  hleg->Leg_Status = tx_status_for_motor(motor);
  HAL_StatusTypeDef ret = SERVO_Send_recv(cmd, fbk,
                                          hleg->GPIOx,
                                          hleg->GPIO_Pin,
                                          hleg->huartx);

  if (ret == HAL_OK && fbk->correct && fbk->motor_id == motor)
  {
    state->angle = fbk->Pos;
    state->angle_valid = Motor_Angle_Valid;
  }
  else
  {
    state->angle_valid = Motor_Angle_Invalid;
    log_motor_io_error(idx, ret, fbk);
  }

  hleg->Leg_Status = Leg_Done;
}

void Leg_Control_Start(void)
{
  for (int i = 0; i < 4; i++)
  {
    if (!Legs[i]->online)
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
    leg_online[i] = Legs[i]->online;
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
