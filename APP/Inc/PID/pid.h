/**
******************************************************************************
  * @file    pid.h
  * @author  Si-Ghost
  * @brief   Head file of pid.c
  ******************************************************************************
  */

#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_PID_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_PID_H

// PID的类型，位置式还是增量式
typedef enum
{
  PID_Position,
  PID_Delta
}PID_Mode;

typedef struct
{
  PID_Mode mode;

  // 目标与实际
  float set;
  float ref;

  // 误差项
  float error_buff[3];
  float diff_buff[3];

  // PID参数
  float kp;
  float ki;
  float kd;

  // 限幅
  float max_out;    // 最大输出
  float max_iout;   // 积分限幅

  // 总输出以及PID三个环节的输出
  float out;
  float pout;
  float iout;
  float dout;
}PID_TypeDef;

void PID_Init(PID_TypeDef *pid, PID_Mode mode, float kp, float ki, float kd, float max_out, float max_iout);
float PID_Calculate(PID_TypeDef *pid, const float set, const float ref);
#endif //PARALLEL_QUADRUPED_ROBOT_STM32_PID_H
