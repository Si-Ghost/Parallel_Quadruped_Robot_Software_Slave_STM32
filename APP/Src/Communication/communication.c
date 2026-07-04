#include "communication.h"
#include <string.h>
#include "crc_ccitt.h"

extern RC_DataTypeDef rc_data;
extern volatile uint32_t last_valid_packet_tick;

static UART_HandleTypeDef *esp32_uart = NULL;

#define RX_BUF_SIZE 128
static uint8_t rx_dma_buf[RX_BUF_SIZE];

static int parse_frame(const uint8_t *data, uint16_t len)
{
    if (len < ESP32_FRAME_LEN)
        return 0;

    if (data[0] != ESP32_HEADER_1 || data[1] != ESP32_HEADER_2)
        return 0;

    if (data[ESP32_FRAME_LEN - 1] != ESP32_FOOTER)
        return 0;

    uint16_t calc_crc = crc_ccitt(0xFFFF, &data[2], 10);
    uint16_t recv_crc = ((uint16_t)data[12] << 8) | data[13];
    if (calc_crc != recv_crc)
        return 0;

    const uint8_t *p = &data[2];
    rc_data.ch0 = (int16_t)(p[0] | ((uint16_t)p[1] << 8));
    rc_data.ch1 = (int16_t)(p[2] | ((uint16_t)p[3] << 8));
    rc_data.ch2 = (int16_t)(p[4] | ((uint16_t)p[5] << 8));
    rc_data.ch3 = (int16_t)(p[6] | ((uint16_t)p[7] << 8));
    rc_data.s1  = p[8];
    rc_data.s2  = p[9];

    if (!((rc_data.ch0 > 363) && (rc_data.ch1 > 363) &&
          (rc_data.ch2 > 363) && (rc_data.ch3 > 363) &&
          (rc_data.ch0 < 1685) && (rc_data.ch1 < 1685) &&
          (rc_data.ch2 < 1685) && (rc_data.ch3 < 1685)))
    {
        rc_data.ch0 = 0;
        rc_data.ch1 = 0;
        rc_data.ch2 = 0;
        rc_data.ch3 = 0;
    } else {
        rc_data.ch0 -= 1024;
        rc_data.ch1 -= 1024;
        rc_data.ch2 -= 1024;
        rc_data.ch3 -= 1024;
    }

    last_valid_packet_tick = HAL_GetTick();
    return ESP32_FRAME_LEN;
}

void Communication_Init(UART_HandleTypeDef *huart)
{
    esp32_uart = huart;

    rc_data.ch0 = 0;
    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;
    rc_data.s1  = 2;
    rc_data.s2  = 3;

    /* DMA接收，IDLE或缓冲区满时触发回调 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buf, RX_BUF_SIZE);
}

void Communication_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART6 || Size == 0) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buf, RX_BUF_SIZE);
        return;
    }

    parse_frame(rx_dma_buf, Size);

    /* re-arm DMA reception */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma_buf, RX_BUF_SIZE);
}

/* ---- Watchdog ---- */
int Communication_IsLinkAlive(void)
{
    return (HAL_GetTick() - last_valid_packet_tick) < WATCHDOG_TIMEOUT_MS;
}

void Communication_ResetWatchdog(void)
{
    last_valid_packet_tick = HAL_GetTick();
}

/* ---- Send to ESP32 ---- */
void Communication_SendByte(uint8_t byte)
{
    if (esp32_uart)
        HAL_UART_Transmit(esp32_uart, &byte, 1, 10);
}

void Communication_SendBytes(const uint8_t *data, uint16_t len)
{
    if (esp32_uart)
        HAL_UART_Transmit(esp32_uart, (uint8_t *)data, len, 100);
}

void Communication_SendString(const char *str)
{
    if (esp32_uart && str)
        HAL_UART_Transmit(esp32_uart, (uint8_t *)str, strlen(str), 1000);
}
