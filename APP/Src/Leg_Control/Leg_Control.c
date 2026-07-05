/**
******************************************************************************
  * @file    Leg_Control.c
  * @author  Si-Ghost
  * @brief   Robot leg low-rate diagnostic control.
  ******************************************************************************
  */

#include "Leg_Control.h"

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
#define LEG_WEB_ANGLE_MIN_RAD      -0.35f
#define LEG_WEB_ANGLE_MAX_RAD       0.35f
#define LEG_WEB_MAX_STEP_RAD        0.03f
#define LEG_WEB_KP                  0.05f
#define LEG_WEB_KW                  0.20f
#define LEG_HANDSHAKE_RETRY         2U

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
    }
    leg_online_state[leg] = 0;
  }
}

void Leg_Control_Handshake(void)
{
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
      cmd->id = motor;
      cmd->mode = 1;
      cmd->T = 0.0f;
      cmd->W = 0.0f;
      cmd->Pos = 0.0f;
      cmd->K_P = 0.0f;
      cmd->K_W = 0.0f;

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

    leg_online_state[leg] = (motor_online_state[motor_index_from_leg_motor(leg, 0)] &&
                             motor_online_state[motor_index_from_leg_motor(leg, 1)]) ? 1 : 0;
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

    Legs[i]->Leg_Status = Leg_TX_M0;
    modify_data(&(Legs[i]->motor_cmd[0]));
    modify_data(&(Legs[i]->motor_cmd[1]));

    HAL_GPIO_WritePin(Legs[i]->GPIOx, Legs[i]->GPIO_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit_DMA(Legs[i]->huartx,
                          (uint8_t *)&(Legs[i]->motor_cmd[0].motor_send_data),
                          sizeof(Legs[i]->motor_cmd[0].motor_send_data));
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
  Leg_Control_Start();
}

int Leg_Control_SetDebugAngle(uint8_t motor_index, float angle_rad)
{
  if (motor_index >= 8)
    return 0;

  uint8_t leg = motor_index / 2;
  uint8_t motor = motor_index % 2;
  if (!leg_online_state[leg])
    return 0;

  MOTOR_send *cmd = &Legs[leg]->motor_cmd[motor];

  float offset = clampf(angle_rad, LEG_WEB_ANGLE_MIN_RAD, LEG_WEB_ANGLE_MAX_RAD);
  float target = Legs[leg]->p_init[motor] + offset;
  float delta = target - cmd->Pos;
  if (delta > LEG_WEB_MAX_STEP_RAD)
    target = cmd->Pos + LEG_WEB_MAX_STEP_RAD;
  else if (delta < -LEG_WEB_MAX_STEP_RAD)
    target = cmd->Pos - LEG_WEB_MAX_STEP_RAD;

  cmd->mode = 1;
  cmd->T = 0.0f;
  cmd->W = 0.0f;
  cmd->Pos = target;
  cmd->K_P = LEG_WEB_KP;
  cmd->K_W = LEG_WEB_KW;
  modify_data(cmd);
  return 1;
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

    hleg->Leg_Status = Leg_TX_M1;
    HAL_UART_Transmit_DMA(hleg->huartx,
                          (uint8_t *)&(hleg->motor_cmd[1].motor_send_data),
                          sizeof(hleg->motor_cmd[1].motor_send_data));
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
