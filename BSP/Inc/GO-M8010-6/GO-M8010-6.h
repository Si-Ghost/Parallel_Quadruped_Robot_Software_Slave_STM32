#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include <stdint.h>
#include "main.h"   // 直接包含main.h，间接包含hal库，增加可移植性
#include "ris_protocol.h"

typedef struct
{
    // 电机控制指令数据包
    ControlData_t motor_send_data;      // 电机控制数据结构体

    // 待发送的各项数据
    uint8_t id;                         // 电机ID
    uint8_t mode;                       // 0:锁定, 1:FOC控制, 2:编码器校准
    float T;                            // 期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                            // 期望关节速度（电机本身的速度）(rad/s)
    float Pos;                          // 期望关节位置（rad）
    float K_P;                          // 关节刚度系数
    float K_W;                          // 关节速度系数
} MOTOR_send;

typedef struct
{
    // 电机接收数据数据包
    MotorData_t motor_recv_data;        // 电机接收数据结构体

    // 数据包的状态
    uint16_t rx_len;                    // 接收到的数据长度
    int correct;                        // 接收数据是否有问题（1完整，0不完整）

    //解读得出的电机数据
    uint8_t motor_id;                   // 电机ID
    uint8_t mode;                       // 0:锁定, 1:FOC控制, 2:编码器校准
    float T;                            // 当前电机实际输出力矩
    float W;							// 当前电机实际角速度
    float Pos;                          // 前电机实际角速度
    int Temp;                           // 温度
    uint8_t MError;                     // 错误码
} MOTOR_recv;

int modify_data(MOTOR_send *motor_s);
int extract_data(MOTOR_recv *motor_r);
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData, GPIO_TypeDef *Port, uint16_t Pin, UART_HandleTypeDef *huart);

#endif
