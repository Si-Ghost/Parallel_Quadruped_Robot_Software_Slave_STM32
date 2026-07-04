#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ESP32_FRAME_LEN     15
#define ESP32_HEADER_1      0xAA
#define ESP32_HEADER_2      0x55
#define ESP32_FOOTER        0x55

#define WATCHDOG_TIMEOUT_MS 500

typedef struct {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s1;
    uint8_t s2;
} RC_DataTypeDef;

void Communication_Init(UART_HandleTypeDef *huart);
void Communication_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);
int Communication_IsLinkAlive(void);
void Communication_ResetWatchdog(void);

void Communication_SendByte(uint8_t byte);
void Communication_SendBytes(const uint8_t *data, uint16_t len);
void Communication_SendString(const char *str);

#endif
