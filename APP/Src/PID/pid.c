/**
******************************************************************************
  * @file    pid.c
  * @author  Si-Ghost
  * @brief   This file provides the function about PID calculation
  ******************************************************************************
  */

#include "pid.h"

#define LimitMax(input, max) input > max ? max : input < -max ? -max : input

/**
 * @brief: PID初始化
 */
void PID_Init(PID_TypeDef *pid, const PID_Mode mode, const float kp, const float ki, const float kd, const float max_out, const float max_iout)
{
    pid->mode = mode;
    pid->error_buff[0] = pid->error_buff[1] = pid->error_buff[2] = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
}

/**
 * @brief PID计算
 * @param pid :PID结构体变量，用之前需要初始化
 * @param set :目标值
 * @param ref :实际值
 * @return
 */
float PID_Calculate(PID_TypeDef *pid, const float set, const float ref)
{
    pid->set = set;
    pid->ref = ref;

    // 更新误差
    pid->error_buff[2] = pid->error_buff[1];
    pid->error_buff[1] = pid->error_buff[0];
    pid->error_buff[0] = set - ref;

    // 位置模式
    if (pid->mode == PID_Position)
    {
        // 更新微分项
        pid->diff_buff[2] = pid->diff_buff[1];
        pid->diff_buff[1] = pid->diff_buff[0];
        pid->diff_buff[0] = pid->error_buff[0] - pid->error_buff[1];

        // PID计算
        pid->pout = pid->kp * pid->error_buff[0];

        pid->iout += pid->ki * pid->error_buff[0];
        pid->iout = LimitMax(pid->iout, pid->max_iout);

        pid->dout = pid->kd * pid->diff_buff[0];

        pid->out = pid->pout + pid->iout + pid->dout;
        pid->out = LimitMax(pid->out, pid->max_out);
    }

    // 增量模式
    else if (pid->mode == PID_Delta)
    {
        // 更新微分项
        pid->diff_buff[2] = pid->diff_buff[1];
        pid->diff_buff[1] = pid->diff_buff[0];
        pid->diff_buff[0] = pid->error_buff[0] - 2.0 * pid->error_buff[1] + pid->error_buff[2];

        // PID计算
        pid->pout = pid->kp * (pid->error_buff[0] - pid->error_buff[1]);
        pid->iout = pid->ki * pid->error_buff[0];
        pid->dout = pid->kd * pid->diff_buff[0];

        pid->out = pid->pout + pid->iout + pid->dout;
        pid->out = LimitMax(pid->out, pid->max_out);
    }

    return pid->out;
}


