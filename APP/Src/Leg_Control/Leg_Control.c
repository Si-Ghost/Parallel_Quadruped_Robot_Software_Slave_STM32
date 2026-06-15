/**
******************************************************************************
  * @file    Leg_Control.c
  * @author  Si-Ghost
  * @brief   This file provides the function about robot leg control
  ******************************************************************************
  */

#include "Leg_Control.h"

extern Leg_HandlerTypeDef Left_Front_Leg;
extern Leg_HandlerTypeDef Right_Front_Leg;
extern Leg_HandlerTypeDef Left_Back_Leg;
extern Leg_HandlerTypeDef Right_Back_Leg;

Leg_HandlerTypeDef* Legs[4] = {
  &Left_Front_Leg,
  &Right_Front_Leg,
  &Left_Back_Leg,
  &Right_Back_Leg,
};

void Leg_Control_Start(void)
{
  // 遍历四条腿
  for (int i = 0; i < 4; i++)
  {
    // 更改状态机状态
    Legs[i]->Leg_Status = Leg_TX_M0;

    // 将数据从浮点数转化为用于发送的定点数
    modify_data(&(Legs[i]->motor_cmd[0]));
    modify_data(&(Legs[i]->motor_cmd[1]));

    // 使能DE
    HAL_GPIO_WritePin(Legs[i]->GPIOx, Legs[i]->GPIO_Pin, GPIO_PIN_SET);

    // 发送数据
    HAL_UART_Transmit_DMA(Legs[i]->huartx, (uint8_t *)&(Legs[i]->motor_cmd[0].motor_send_data), sizeof(Legs[i]->motor_cmd[0].motor_send_data));
  }
}

static void Leg_Tx_Handler(Leg_HandlerTypeDef *hleg)
{
  // 开启接受
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_RESET);

  // 如果是完成了电机0的发送，那么进行电机0的数据接受
  if (hleg->Leg_Status == Leg_TX_M0)
  {
    hleg->Leg_Status = Leg_RX_M0;
    HAL_UARTEx_ReceiveToIdle_DMA(hleg->huartx, (uint8_t *)&(hleg->motor_data[0].motor_recv_data), sizeof(hleg->motor_data[0].motor_recv_data));
  }

  // 如果是完成了电机1的发送，那么进行电机1的数据接受
  if (hleg->Leg_Status == Leg_TX_M1)
  {
    hleg->Leg_Status = Leg_RX_M1;
    HAL_UARTEx_ReceiveToIdle_DMA(hleg->huartx, (uint8_t *)&(hleg->motor_data[1].motor_recv_data), sizeof(hleg->motor_data[1].motor_recv_data));
  }
}

void Leg_TxCpltCallback(UART_HandleTypeDef *huart)
{
  for (int i = 0; i < 4; i++)
  {
    if (huart->Instance == Legs[i]->huartx->Instance)
    {
      Leg_Tx_Handler(Legs[i]);
      break;
    }
  }
}

static void Leg_Rx_Handler(Leg_HandlerTypeDef *hleg)
{
  HAL_GPIO_WritePin(hleg->GPIOx, hleg->GPIO_Pin, GPIO_PIN_SET);

  // 如果是完成了电机0的接收，那么发送电机1的指令
  if (hleg->Leg_Status == Leg_RX_M0)
  {
    // 解读数据
    if (hleg->motor_data[0].motor_recv_data.head[0] == 0xFE && hleg->motor_data[0].motor_recv_data.head[1] == 0xEE)
    {
      hleg->motor_data[0].correct = 1;
      extract_data(&hleg->motor_data[0]);
    }

    // 发送数据
    hleg->Leg_Status = Leg_TX_M1;
    HAL_UART_Transmit_DMA(hleg->huartx, (uint8_t *)&(hleg->motor_cmd[1].motor_send_data), sizeof(hleg->motor_cmd[1].motor_send_data));
  }

  // 如果完成了电机1的接收，那么循环完成
  if (hleg->Leg_Status == Leg_RX_M1)
  {
    // 解读数据
    if (hleg->motor_data[1].motor_recv_data.head[0] == 0xFE && hleg->motor_data[1].motor_recv_data.head[1] == 0xEE)
    {
      hleg->motor_data[1].correct = 1;
      extract_data(&hleg->motor_data[1]);
    }

    hleg->Leg_Status = Leg_Done;
  }
}

void Leg_RxCpltCallback(UART_HandleTypeDef *huart)
{
  for (int i = 0; i < 4; i++)
  {
    if (huart->Instance == Legs[i]->huartx->Instance)
    {
      Leg_Rx_Handler(Legs[i]);
      break;
    }
  }
}