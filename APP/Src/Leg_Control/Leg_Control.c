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
#define LEG_WEB_MAX_STEP_RAD        0.15f
#define LEG_WEB_KP                  0.8f
#define LEG_WEB_KW                  0.01f
#define LEG_HANDSHAKE_KW            0.05f
#define LEG_HANDSHAKE_RETRY         2U
#define LEG_DEBUG_LOG_PERIOD_MS     250U

#define LEG_HS_OK                   0U
#define LEG_HS_TIMEOUT              1U
#define LEG_HS_UART_ERROR           2U
#define LEG_HS_BAD_ID               3U

static uint32_t last_service_tick = 0;
static float motor_angles[8] = {0};
static uint8_t motor_angle_valid[8] = {0};
static uint8_t motor_online_state[8] = {0};
static uint8_t leg_online_state[4] = {0};
static uint8_t motor_handshake_error[8] = {0};
static float motor_target_offset[8] = {0};
static uint8_t motor_target_active[8] = {0};
static uint32_t motor_debug_last_log_tick[8] = {0};
static volatile uint8_t handshake_requested = 0;

static float clampf(float value, float min_value, float max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
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

static uint8_t motor_is_online(uint8_t leg, uint8_t motor)
{
  if (leg >= 4 || motor >= 2)
    return 0;

  return motor_online_state[motor_index_from_leg_motor(leg, motor)] != 0;
}

static uint8_t first_online_motor(uint8_t leg)
{
  if (motor_is_online(leg, 0))
    return 0;
  if (motor_is_online(leg, 1))
    return 1;
  return 0xFF;
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

  char buf[192];
  int len = snprintf(buf, sizeof(buf), "MOTOR_CMD %s idx=%u off=", tag, motor_index);
  len = append_fixed4(buf, sizeof(buf), len, motor_target_offset[motor_index]);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " cmd=");
  len = append_fixed4(buf, sizeof(buf), len, cmd->Pos);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " target=");
  len = append_fixed4(buf, sizeof(buf), len, desired);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " fbk=");
  len = append_fixed4(buf, sizeof(buf), len, motor_angles[motor_index]);
  if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " delta=");
  len = append_fixed4(buf, sizeof(buf), len, delta);
  if (len > 0)
  {
    int written = snprintf(&buf[len], sizeof(buf) - (size_t)len,
                           " raw_pos=%ld raw_kp=%d raw_kw=%d active=%u\r\n",
                           (long)cmd->motor_send_data.comd.pos_des,
                           (int)cmd->motor_send_data.comd.k_pos,
                           (int)cmd->motor_send_data.comd.k_spd,
                           motor_target_active[motor_index]);
    if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
      return;
    len += written;
  }

  if (len > 0 && len < (int)sizeof(buf))
    Communication_SendString(buf);
}

static int apply_debug_target(uint8_t motor_index, uint8_t force_log)
{
  if (motor_index >= 8 || !motor_online_state[motor_index])
    return 0;

  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];

  float desired = Legs[leg]->p_init[motor] + motor_target_offset[motor_index];
  float delta = desired - cmd->Pos;
  if (delta > LEG_WEB_MAX_STEP_RAD)
  {
    cmd->Pos += LEG_WEB_MAX_STEP_RAD;
  }
  else if (delta < -LEG_WEB_MAX_STEP_RAD)
  {
    cmd->Pos -= LEG_WEB_MAX_STEP_RAD;
  }
  else
  {
    cmd->Pos = desired;
    motor_target_active[motor_index] = 0;
  }

  cmd->mode = 1;
  cmd->T = 0.0f;
  cmd->W = 0.0f;
  cmd->K_P = LEG_WEB_KP;
  cmd->K_W = LEG_WEB_KW;
  modify_data(cmd);

  uint32_t now = HAL_GetTick();
  if (force_log || !motor_target_active[motor_index] ||
      (now - motor_debug_last_log_tick[motor_index]) >= LEG_DEBUG_LOG_PERIOD_MS)
  {
    motor_debug_last_log_tick[motor_index] = now;
    log_debug_target(motor_index,
                     motor_target_active[motor_index] ? "step" : "done",
                     desired,
                     desired - cmd->Pos);
  }

  return 1;
}

static void update_debug_targets(void)
{
  for (uint8_t idx = 0; idx < 8; idx++)
  {
    if (motor_target_active[idx])
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
      uint8_t idx = motor_index_from_leg_motor(leg, motor);
      motor_angles[idx] = 0.0f;
      motor_angle_valid[idx] = 0;
      motor_online_state[idx] = 0;
      motor_handshake_error[idx] = LEG_HS_TIMEOUT;
      motor_target_offset[idx] = 0.0f;
      motor_target_active[idx] = 0;
      motor_debug_last_log_tick[idx] = 0;
    }
    leg_online_state[leg] = 0;
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
    leg_online_state[leg] = 0;

    for (uint8_t motor = 0; motor < 2; motor++)
    {
      uint8_t idx = motor_index_from_leg_motor(leg, motor);
      MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];
      MOTOR_recv *fbk = &Legs[leg]->motor_data[motor];

      motor_online_state[idx] = 0;
      motor_angle_valid[idx] = 0;
      motor_handshake_error[idx] = LEG_HS_TIMEOUT;
      motor_target_offset[idx] = 0.0f;
      motor_target_active[idx] = 0;
      motor_debug_last_log_tick[idx] = 0;
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
          motor_online_state[idx] = 1;
          motor_angle_valid[idx] = 1;
          motor_handshake_error[idx] = LEG_HS_OK;
          motor_angles[idx] = fbk->Pos;
          Legs[leg]->p_init[motor] = fbk->Pos;
          cmd->Pos = fbk->Pos;
          modify_data(cmd);
          break;
        }
        else if (ret == HAL_TIMEOUT)
        {
          motor_handshake_error[idx] = LEG_HS_TIMEOUT;
        }
        else if (ret == HAL_OK)
        {
          motor_handshake_error[idx] = LEG_HS_BAD_ID;
        }
        else
        {
          motor_handshake_error[idx] = LEG_HS_UART_ERROR;
        }
      }
    }

    leg_online_state[leg] = (motor_is_online(leg, 0) || motor_is_online(leg, 1)) ? 1 : 0;
    Legs[leg]->Leg_Status = Leg_Idle;
    HAL_GPIO_WritePin(Legs[leg]->GPIOx, Legs[leg]->GPIO_Pin, GPIO_PIN_RESET);
  }
}

void Leg_Control_RequestHandshake(void)
{
  handshake_requested = 1;
}

void Leg_Control_Start(void)
{
  for (int i = 0; i < 4; i++)
  {
    if (!leg_online_state[i])
      continue;

    if (Legs[i]->Leg_Status != Leg_Idle && Legs[i]->Leg_Status != Leg_Done)
      continue;

    uint8_t motor = first_online_motor((uint8_t)i);
    if (motor >= 2)
      continue;

    Legs[i]->Leg_Status = tx_status_for_motor(motor);
    modify_data(&(Legs[i]->motor_cmd[motor]));

    HAL_GPIO_WritePin(Legs[i]->GPIOx, Legs[i]->GPIO_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(Legs[i]->huartx,
                          (uint8_t *)&(Legs[i]->motor_cmd[motor].motor_send_data),
                          sizeof(Legs[i]->motor_cmd[motor].motor_send_data));
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
  if (motor_index >= 8)
    return 0;

  if (!motor_online_state[motor_index])
    return 0;

  motor_target_offset[motor_index] = clampf(angle_rad, LEG_WEB_ANGLE_MIN_RAD, LEG_WEB_ANGLE_MAX_RAD);
  motor_target_active[motor_index] = 1;
  return apply_debug_target(motor_index, 1);
}

void Leg_Control_GetAngles(float angles[8], uint8_t valid[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
  {
    angles[i] = motor_angles[i];
    valid[i] = motor_angle_valid[i];
  }
  __enable_irq();
}

void Leg_Control_GetOnline(uint8_t motor_online[8], uint8_t leg_online[4])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
    motor_online[i] = motor_online_state[i];
  for (uint8_t i = 0; i < 4; i++)
    leg_online[i] = leg_online_state[i];
  __enable_irq();
}

void Leg_Control_GetHandshakeErrors(uint8_t motor_error[8])
{
  __disable_irq();
  for (uint8_t i = 0; i < 8; i++)
    motor_error[i] = motor_handshake_error[i];
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
        motor_angles[idx] = hleg->motor_data[0].Pos;
        motor_angle_valid[idx] = (motor_online_state[idx] && hleg->motor_data[0].correct) ? 1 : 0;
      }
    }
    else if (idx < 8)
    {
      motor_angle_valid[idx] = 0;
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
        motor_angles[idx] = hleg->motor_data[1].Pos;
        motor_angle_valid[idx] = (motor_online_state[idx] && hleg->motor_data[1].correct) ? 1 : 0;
      }
    }
    else if (idx < 8)
    {
      motor_angle_valid[idx] = 0;
    }

    hleg->Leg_Status = Leg_Done;
  }
}
