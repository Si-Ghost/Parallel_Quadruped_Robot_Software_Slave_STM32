#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ESP32_FRAME_LEN     15
#define ESP32_HEADER_1      0xAA
#define ESP32_HEADER_2      0x55
#define ESP32_FOOTER        0x55

#define ESP32_HANDSHAKE_TIMEOUT_MS 3000
#define ESP32_WATCHDOG_TIMEOUT_MS 500

/* 设为 1 时，会把 UART6 接收状态回显给 ESP32，方便网页侧链路测试。 */
#define ESP32_LINK_ECHO_TEST 0

typedef struct {
    int16_t ch0;  /* 居中并处理过死区的遥控通道 0。 */
    int16_t ch1;  /* 居中并处理过死区的遥控通道 1。 */
    int16_t ch2;  /* 居中并处理过死区的遥控通道 2。 */
    int16_t ch3;  /* 居中并处理过死区的遥控通道 3。 */
    uint8_t s1;   /* 来自 ESP32 的主模式开关，合法范围 1..3。 */
    uint8_t s2;   /* 来自 ESP32 的副模式开关，合法范围 1..3。 */
} RC_DataTypeDef;

void Communication_Init(UART_HandleTypeDef *huart);
void Communication_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
int  Communication_IsHandshakeDone(void);
int  Communication_IsLinkAlive(void);
void Communication_SetSafeRCData(void);
void Communication_ResetWatchdog(void);
void Communication_Task(void);

void Communication_SendByte(uint8_t byte);
void Communication_SendBytes(const uint8_t *data, uint16_t len);
void Communication_SendString(const char *str);

void Communication_NotifyTxComplete(void);
void Communication_SendMotorAngles(void);
void Communication_SendMotorStatus(void);

#endif
