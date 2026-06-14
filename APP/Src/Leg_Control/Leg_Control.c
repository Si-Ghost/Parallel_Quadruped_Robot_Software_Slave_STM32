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

void Leg_Control_Start(void)
{
  // 更改状态机状态
  Left_Front_Leg.Leg_Status = Leg_TX_M0;
  Right_Front_Leg.Leg_Status = Leg_TX_M0;
  Left_Back_Leg.Leg_Status = Leg_TX_M0;
  Right_Back_Leg.Leg_Status = Leg_TX_M0;

  // 调整要发送的数据
  modify_data(&(Left_Front_Leg.motor_cmd[0]));
  modify_data(&(Right_Front_Leg.motor_cmd[0]));
  modify_data(&(Left_Back_Leg.motor_cmd[0]));
  modify_data(&(Right_Back_Leg.motor_cmd[0]));

  modify_data(&(Left_Front_Leg.motor_cmd[1]));
  modify_data(&(Right_Front_Leg.motor_cmd[1]));
  modify_data(&(Left_Back_Leg.motor_cmd[1]));
  modify_data(&(Right_Back_Leg.motor_cmd[1]));

  // 使能DE
  HAL_GPIO_WritePin(Left_Front_Leg.GPIOx, Left_Front_Leg.GPIO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Right_Front_Leg.GPIOx, Right_Front_Leg.GPIO_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(Left_Back_Leg.GPIOx, Left_Back_Leg.GPIO_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(Right_Back_Leg.GPIOx, Right_Back_Leg.GPIO_Pin,GPIO_PIN_SET);

  HAL_UART_Transmit_DMA(Left_Front_Leg.huartx, (uint8_t *)&(Left_Front_Leg.motor_cmd[0].motor_send_data),sizeof(Left_Front_Leg.motor_cmd[0].motor_send_data));
  HAL_UART_Transmit_DMA(Right_Front_Leg.huartx, (uint8_t *)&(Right_Front_Leg.motor_cmd[0].motor_send_data), sizeof(Right_Front_Leg.motor_cmd[0].motor_send_data));
  HAL_UART_Transmit_DMA(Left_Back_Leg.huartx, (uint8_t *)&(Left_Back_Leg.motor_cmd[0].motor_send_data), sizeof(Left_Back_Leg.motor_cmd[0].motor_send_data));
  HAL_UART_Transmit_DMA(Right_Back_Leg.huartx, (uint8_t *)&(Right_Back_Leg.motor_cmd[0].motor_send_data), sizeof(Right_Back_Leg.motor_cmd[0].motor_send_data));
}

void Leg_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // 左前腿
  if (huart->Instance == USART2)
  {
    HAL_GPIO_WritePin(Left_Front_Leg.GPIOx, Left_Front_Leg.GPIO_Pin, GPIO_PIN_RESET);
    if (Left_Front_Leg.Leg_Status == Leg_TX_M0)
    {
      Left_Front_Leg.Leg_Status = Leg_RX_M0;
      HAL_UART_Receive_DMA(Left_Front_Leg.huartx, (uint8_t *)&(Left_Front_Leg.motor_data[0].motor_recv_data), sizeof(Left_Front_Leg.motor_data[0].motor_recv_data));
    }
    if (Left_Front_Leg.Leg_Status == Leg_TX_M1)
    {
      Left_Front_Leg.Leg_Status = Leg_RX_M1;
      HAL_UART_Receive_DMA(Left_Front_Leg.huartx, (uint8_t *)&(Left_Front_Leg.motor_data[1].motor_recv_data), sizeof(Left_Front_Leg.motor_data[1].motor_recv_data));
    }
  }

  // 右前腿
  if (huart->Instance == UART8)
  {
    HAL_GPIO_WritePin(Right_Front_Leg.GPIOx, Right_Front_Leg.GPIO_Pin, GPIO_PIN_RESET);
    if (Right_Front_Leg.Leg_Status == Leg_TX_M0)
    {
      Right_Front_Leg.Leg_Status = Leg_RX_M0;
      HAL_UART_Receive_DMA(Right_Front_Leg.huartx, (uint8_t *)&(Right_Front_Leg.motor_data[0].motor_recv_data), sizeof(Right_Front_Leg.motor_data[0].motor_recv_data));
    }
    if (Right_Front_Leg.Leg_Status == Leg_TX_M1)
    {
      Right_Front_Leg.Leg_Status = Leg_RX_M1;
      HAL_UART_Receive_DMA(Right_Front_Leg.huartx, (uint8_t *)&(Right_Front_Leg.motor_data[1].motor_recv_data), sizeof(Right_Front_Leg.motor_data[1].motor_recv_data));
    }
  }

  // 左后腿
  if (huart->Instance == UART7)
  {
    HAL_GPIO_WritePin(Right_Back_Leg.GPIOx, Right_Back_Leg.GPIO_Pin, GPIO_PIN_RESET);
    if (Right_Back_Leg.Leg_Status == Leg_TX_M0)
    {
      Right_Back_Leg.Leg_Status = Leg_RX_M0;
      HAL_UART_Receive_DMA(Right_Back_Leg.huartx, (uint8_t *)&(Right_Back_Leg.motor_data[0].motor_recv_data), sizeof(Right_Back_Leg.motor_data[0].motor_recv_data));
    }
    if (Right_Back_Leg.Leg_Status == Leg_TX_M1)
    {
      Right_Back_Leg.Leg_Status = Leg_RX_M1;
      HAL_UART_Receive_DMA(Right_Back_Leg.huartx, (uint8_t *)&(Right_Back_Leg.motor_data[1].motor_recv_data), sizeof(Right_Back_Leg.motor_data[1].motor_recv_data));
    }
  }

  // 右后腿
  if (huart->Instance == UART5)
  {
    HAL_GPIO_WritePin(Right_Front_Leg.GPIOx, Right_Front_Leg.GPIO_Pin, GPIO_PIN_RESET);
    if (Right_Front_Leg.Leg_Status == Leg_TX_M0)
    {
      Right_Front_Leg.Leg_Status = Leg_RX_M0;
      HAL_UART_Receive_DMA(Right_Front_Leg.huartx, (uint8_t *)&(Right_Front_Leg.motor_data[0].motor_recv_data), sizeof(Right_Front_Leg.motor_data[0].motor_recv_data));
    }
    if (Right_Front_Leg.Leg_Status == Leg_TX_M1)
    {
      Right_Front_Leg.Leg_Status = Leg_RX_M1;
      HAL_UART_Receive_DMA(Right_Front_Leg.huartx, (uint8_t *)&(Right_Front_Leg.motor_data[1].motor_recv_data), sizeof(Right_Front_Leg.motor_data[1].motor_recv_data));
    }
  }
}

void Leg_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 左前腿
  if (huart->Instance == USART2)
  {
    HAL_GPIO_WritePin(Left_Front_Leg.GPIOx, Left_Front_Leg.GPIO_Pin, GPIO_PIN_SET);
    if (Left_Front_Leg.Leg_Status == Leg_RX_M0)
    {
      Left_Front_Leg.Leg_Status = Leg_TX_M1;
      HAL_UART_Transmit_DMA(Left_Front_Leg.huartx, (uint8_t *)&(Left_Front_Leg.motor_cmd[1].motor_send_data),sizeof(Left_Front_Leg.motor_cmd[1].motor_send_data));

      // 解读数据
      if (Left_Front_Leg.motor_data[0].motor_recv_data.head[0] == 0xFE && Left_Front_Leg.motor_data[0].motor_recv_data.head[1] == 0xEE)
      {
        Left_Front_Leg.motor_data[0].correct = 1;
        extract_data(&Left_Front_Leg.motor_data[0]);
      }
    }

    if (Left_Front_Leg.Leg_Status == Leg_RX_M1)
    {
      Left_Front_Leg.Leg_Status = Leg_Done;

      // 解读数据
      if (Left_Front_Leg.motor_data[1].motor_recv_data.head[0] == 0xFE && Left_Front_Leg.motor_data[1].motor_recv_data.head[1] == 0xEE)
      {
        Left_Front_Leg.motor_data[1].correct = 1;
        extract_data(&Left_Front_Leg.motor_data[1]);
      }
    }
  }

  // 右前腿
  if (huart->Instance == UART8)
  {
    HAL_GPIO_WritePin(Right_Front_Leg.GPIOx, Right_Front_Leg.GPIO_Pin, GPIO_PIN_SET);
    if (Right_Front_Leg.Leg_Status == Leg_RX_M0)
    {
      Right_Front_Leg.Leg_Status = Leg_TX_M1;
      HAL_UART_Transmit_DMA(Right_Front_Leg.huartx, (uint8_t *)&(Right_Front_Leg.motor_cmd[1].motor_send_data), sizeof(Right_Front_Leg.motor_cmd[1].motor_send_data));

      // 解读数据
      if (Right_Front_Leg.motor_data[0].motor_recv_data.head[0] == 0xFE && Right_Front_Leg.motor_data[0].motor_recv_data.head[1] == 0xEE)
      {
        Right_Front_Leg.motor_data[0].correct = 1;
        extract_data(&Right_Front_Leg.motor_data[0]);
      }
    }
    if (Right_Front_Leg.Leg_Status == Leg_RX_M1)
    {
      Right_Front_Leg.Leg_Status = Leg_Done;

      // 解读数据
      if (Right_Front_Leg.motor_data[1].motor_recv_data.head[0] == 0xFE && Right_Front_Leg.motor_data[1].motor_recv_data.head[1] == 0xEE)
      {
        Right_Front_Leg.motor_data[1].correct = 1;
        extract_data(&Right_Front_Leg.motor_data[1]);
      }
    }
  }

  // 左后腿
  if (huart->Instance == UART7)
  {
    HAL_GPIO_WritePin(Left_Back_Leg.GPIOx, Left_Back_Leg.GPIO_Pin, GPIO_PIN_SET);
    if (Left_Back_Leg.Leg_Status == Leg_RX_M0)
    {
      Left_Back_Leg.Leg_Status = Leg_TX_M1;
      HAL_UART_Transmit_DMA(Left_Back_Leg.huartx, (uint8_t *)&(Left_Back_Leg.motor_cmd[1].motor_send_data), sizeof(Left_Back_Leg.motor_cmd[1].motor_send_data));

      // 解读数据
      if (Left_Back_Leg.motor_data[0].motor_recv_data.head[0] == 0xFE && Left_Back_Leg.motor_data[0].motor_recv_data.head[1] == 0xEE)
      {
        Left_Back_Leg.motor_data[0].correct = 1;
        extract_data(&Left_Back_Leg.motor_data[0]);
      }
    }
    if (Left_Back_Leg.Leg_Status == Leg_RX_M1)
    {
      Left_Back_Leg.Leg_Status = Leg_Done;

      // 解读数据
      if (Left_Back_Leg.motor_data[1].motor_recv_data.head[0] == 0xFE && Left_Back_Leg.motor_data[1].motor_recv_data.head[1] == 0xEE)
      {
        Left_Back_Leg.motor_data[1].correct = 1;
        extract_data(&Left_Back_Leg.motor_data[1]);
      }
    }
  }

  // 右后腿
  if (huart->Instance == UART5)
  {
    HAL_GPIO_WritePin(Right_Back_Leg.GPIOx, Right_Back_Leg.GPIO_Pin, GPIO_PIN_SET);
    if (Right_Back_Leg.Leg_Status == Leg_RX_M0)
    {
      Right_Back_Leg.Leg_Status = Leg_TX_M1;
      HAL_UART_Transmit_DMA(Right_Back_Leg.huartx, (uint8_t *)&(Right_Back_Leg.motor_cmd[1].motor_send_data), sizeof(Right_Back_Leg.motor_cmd[1].motor_send_data));

      // 解读数据
      if (Right_Back_Leg.motor_data[0].motor_recv_data.head[0] == 0xFE && Right_Back_Leg.motor_data[0].motor_recv_data.head[1] == 0xEE)
      {
        Right_Back_Leg.motor_data[0].correct = 1;
        extract_data(&Right_Back_Leg.motor_data[0]);
      }
    }
    if (Right_Back_Leg.Leg_Status == Leg_RX_M1)
    {
      Right_Back_Leg.Leg_Status = Leg_Done;

      // 解读数据
      if (Right_Back_Leg.motor_data[1].motor_recv_data.head[0] == 0xFE && Right_Back_Leg.motor_data[1].motor_recv_data.head[1] == 0xEE)
      {
        Right_Back_Leg.motor_data[1].correct = 1;
        extract_data(&Right_Back_Leg.motor_data[1]);
      }
    }
  }
}