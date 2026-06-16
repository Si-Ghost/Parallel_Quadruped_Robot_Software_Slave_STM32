#include "../../Inc/GO-M8010-6/GO-M8010-6.h"
#include "../../Inc/GO-M8010-6/crc_ccitt.h"
#include "stdio.h"

#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 }

/**
 * @brief 调整发送的数据
 * @param motor_s: 用于接收发送的数据
 * @retval 如果成功则返回0
 */
int modify_data(MOTOR_send *motor_s)
{
	// 对数据进行限幅
	SATURATE(motor_s->id,   0,    15);
	SATURATE(motor_s->mode, 0,    2);
	SATURATE(motor_s->K_P,  0.0f,   25.599f);
	SATURATE(motor_s->K_W,  0.0f,   25.599f);
	SATURATE(motor_s->T,   -127.99f,  127.99f);
	SATURATE(motor_s->W,   -804.00f,  804.00f);
	SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

	// 指定包长度和包头
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;

	// 将可读的数据转化为要传输的数据
    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
	motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
	motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768;
	motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
	motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}

/**
 * @brief 解压发送来的数据
 * @param motor_r: 用于接收发来的数据
 * @retval 如果成功则返回1，失败返回0
 */
int extract_data(MOTOR_recv *motor_r)
{
	// CRC校验，成功则继续执行，失败则退出
    if(motor_r->motor_recv_data.CRC16 != crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14))
    {
        motor_r->correct = 0;
        return motor_r->correct;
    }

	// 读取数据
    motor_r->motor_id = motor_r->motor_recv_data.mode.id;
    motor_r->mode = motor_r->motor_recv_data.mode.status;
	motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
	motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed/256)*6.2832f ;
	motor_r->Pos = 6.2832f*((float)motor_r->motor_recv_data.fbk.pos) / 32768;
    motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
    motor_r->MError = motor_r->motor_recv_data.fbk.MError;
	motor_r->correct = 1;
    return motor_r->correct;
}

/**
 * @brief 发送和接收电机信息
 * @param pData: 发送的数据
 * @param rData: 接受的数据
 * @param Port: 用于控制RS485使能的引脚端口
 * @param Pin: 用于控制RS485使能的引脚
 * @param huart: 用以发送的句柄
 * @retval 如果成功则返回1，失败返回0
 */
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData, GPIO_TypeDef *Port, uint16_t Pin, UART_HandleTypeDef *huart)
{
    uint16_t rxlen = 0;

	//调整数据然后发送并等待接收
    modify_data(pData);

	//设置为发送，然后发送数据
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(huart, (uint8_t *)&(pData->motor_send_data), sizeof(pData->motor_send_data), 10);

	//设置为接收模式，然后接收数据
	HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET);
	HAL_UARTEx_ReceiveToIdle(huart, (uint8_t *)&(rData->motor_recv_data), sizeof(rData->motor_recv_data), &rxlen, 10);
		
	// 接收处理，如果数据长度为零则是超时，不对就是错误
    if(rxlen == 0)
      return HAL_TIMEOUT;

    if(rxlen != sizeof(rData->motor_recv_data))
			return HAL_ERROR;

	//
    uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
    if(rp[0] == 0xFE && rp[1] == 0xEE)
    {
        rData->correct = 1;
        extract_data(rData);
        return HAL_OK;
    }
    
    return HAL_ERROR;
}