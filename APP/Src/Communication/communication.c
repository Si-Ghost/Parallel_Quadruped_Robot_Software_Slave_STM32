#include "communication.h"
#include <string.h>
#include <stdio.h>
#include "crc_ccitt.h"
#include "Leg_Control.h"

extern RC_DataTypeDef rc_data;
extern volatile uint32_t last_valid_packet_tick;

static UART_HandleTypeDef *esp32_uart = NULL;
static UART_HandleTypeDef *debug_uart = NULL;

#define RX_BUF_SIZE 128
static uint8_t rx_it_buf[RX_BUF_SIZE];

#define TX_IT_BUF_SIZE 256
static uint8_t tx_it_buf[TX_IT_BUF_SIZE];
static volatile uint8_t tx_it_busy = 0;

#define DEBUG_RX_BUF_SIZE 256
static uint8_t debug_rx_buf[DEBUG_RX_BUF_SIZE];

static volatile uint8_t rc_data_updated = 0;
static volatile uint8_t handshake_done = 0;
static uint32_t last_hello_tick = 0;
static uint32_t last_echo_tick = 0;
static volatile uint16_t diag_last_calc_crc = 0;
static volatile uint16_t diag_last_recv_crc = 0;

static const char esp32_hello[] = "ESP32_HELLO";
static const char stm32_ack[] = "STM32_ACK\n";
static const char motor_set_cmd[] = "MOTOR_SET";

#define RC_RAW_MIN       0
#define RC_RAW_MAX       2047
#define RC_CENTER        1024
#define RC_DEADZONE      363

/* ---- 诊断用变量 ---- */
volatile uint32_t diag_usart6_rx_count = 0;   // USART6 RX 回调触发次数
volatile uint16_t diag_last_rx_size   = 0;    // 最后一次收到的字节数
volatile uint8_t  diag_rx_snapshot[16] = {0}; // 最后一次收到数据的前16字节快照

static void send_debug_line(const char *str)
{
    if (debug_uart && str) {
        HAL_UART_Transmit(debug_uart, (uint8_t *)str, strlen(str), 100);
    }
}

static void restart_esp32_rx(void)
{
    if (esp32_uart) {
        HAL_UARTEx_ReceiveToIdle_IT(esp32_uart, rx_it_buf, RX_BUF_SIZE);
    }
}

static void send_esp32_rx_echo(uint16_t size, int parsed_frame)
{
#if ESP32_LINK_ECHO_TEST
    uint32_t now = HAL_GetTick();
    if (now - last_echo_tick < 500) {
        return;
    }
    last_echo_tick = now;

    char buf[80];
    int len;
    if (parsed_frame) {
        len = snprintf(buf, sizeof(buf),
                       "STM32_RX cnt=%lu size=%u frame=1\r\n",
                       diag_usart6_rx_count,
                       (unsigned int)size);
    } else {
        len = snprintf(buf, sizeof(buf),
                       "STM32_RX cnt=%lu size=%u frame=0 head=%02X%02X tail=%02X crc=%04X/%04X\r\n",
                       diag_usart6_rx_count,
                       (unsigned int)size,
                       diag_rx_snapshot[0],
                       diag_rx_snapshot[1],
                       size > 0 ? diag_rx_snapshot[(size < 16 ? size : 16) - 1] : 0,
                       diag_last_recv_crc,
                       diag_last_calc_crc);
    }
    if (len > 0) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
#else
    (void)size;
    (void)parsed_frame;
#endif
}

void Communication_SetSafeRCData(void)
{
    __disable_irq();
    rc_data.ch0 = 0;
    rc_data.ch1 = 0;
    rc_data.ch2 = 0;
    rc_data.ch3 = 0;
    rc_data.s1  = 2;
    rc_data.s2  = 3;
    __enable_irq();
}

static void handle_hello(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(esp32_hello) - 1)
        return;

    for (uint16_t i = 0; i + sizeof(esp32_hello) - 1 <= len; i++) {
        if (memcmp(&data[i], esp32_hello, sizeof(esp32_hello) - 1) == 0) {
            uint8_t first_handshake = !handshake_done;
            handshake_done = 1;
            last_hello_tick = HAL_GetTick();
            Communication_SendBytes((const uint8_t *)stm32_ack, sizeof(stm32_ack) - 1);
            if (first_handshake) {
                send_debug_line("[LINK] ESP32 handshake OK\r\n");
            }
            return;
        }
    }
}

static void handle_motor_debug_command(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(motor_set_cmd) - 1)
        return;

    for (uint16_t i = 0; i + sizeof(motor_set_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_set_cmd, sizeof(motor_set_cmd) - 1) == 0) {
            int motor = -1;
            float angle = 0.0f;
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf))
                copy_len = sizeof(cmd_buf) - 1;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';

            if (sscanf(cmd_buf, "MOTOR_SET %d %f", &motor, &angle) == 2) {
                if (motor >= 0 && motor < 8) {
                    Leg_Control_SetDebugAngle((uint8_t)motor, angle);
                }
            }
            return;
        }
    }
}

static int parse_frame(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i + ESP32_FRAME_LEN <= len; i++)
    {
        if (data[i] != ESP32_HEADER_1 || data[i + 1] != ESP32_HEADER_2)
            continue;
        if (data[i + ESP32_FRAME_LEN - 1] != ESP32_FOOTER)
            continue;

        uint16_t calc_crc = crc_ccitt(0xFFFF, &data[i + 2], 10);
        uint16_t recv_crc = ((uint16_t)data[i + 12] << 8) | data[i + 13];
        diag_last_calc_crc = calc_crc;
        diag_last_recv_crc = recv_crc;
        if (calc_crc != recv_crc)
            continue;

        const uint8_t *p = &data[i + 2];
        RC_DataTypeDef tmp;
        tmp.ch0 = (int16_t)(p[0] | ((uint16_t)p[1] << 8));
        tmp.ch1 = (int16_t)(p[2] | ((uint16_t)p[3] << 8));
        tmp.ch2 = (int16_t)(p[4] | ((uint16_t)p[5] << 8));
        tmp.ch3 = (int16_t)(p[6] | ((uint16_t)p[7] << 8));
        tmp.s1  = p[8];
        tmp.s2  = p[9];

        if (!((tmp.ch0 >= RC_RAW_MIN) && (tmp.ch1 >= RC_RAW_MIN) &&
              (tmp.ch2 >= RC_RAW_MIN) && (tmp.ch3 >= RC_RAW_MIN) &&
              (tmp.ch0 <= RC_RAW_MAX) && (tmp.ch1 <= RC_RAW_MAX) &&
              (tmp.ch2 <= RC_RAW_MAX) && (tmp.ch3 <= RC_RAW_MAX) &&
              (tmp.s1 >= 1) && (tmp.s1 <= 3) &&
              (tmp.s2 >= 1) && (tmp.s2 <= 3)))
        {
            Communication_SetSafeRCData();
            return 0;
        }
        else
        {
            tmp.ch0 -= RC_CENTER;
            tmp.ch1 -= RC_CENTER;
            tmp.ch2 -= RC_CENTER;
            tmp.ch3 -= RC_CENTER;

            if (tmp.ch0 >= -RC_DEADZONE && tmp.ch0 <= RC_DEADZONE) tmp.ch0 = 0;
            if (tmp.ch1 >= -RC_DEADZONE && tmp.ch1 <= RC_DEADZONE) tmp.ch1 = 0;
            if (tmp.ch2 >= -RC_DEADZONE && tmp.ch2 <= RC_DEADZONE) tmp.ch2 = 0;
            if (tmp.ch3 >= -RC_DEADZONE && tmp.ch3 <= RC_DEADZONE) tmp.ch3 = 0;
        }

        __disable_irq();
        rc_data = tmp;
        rc_data_updated = 1;
        __enable_irq();
        handshake_done = 1;
        last_valid_packet_tick = HAL_GetTick();
        return i + ESP32_FRAME_LEN;
    }
    return 0;
}

void Communication_Init(UART_HandleTypeDef *huart)
{
    esp32_uart = huart;
    tx_it_busy = 0;
    handshake_done = 0;
    last_hello_tick = HAL_GetTick();

    Communication_SetSafeRCData();
    last_valid_packet_tick = 0;

    restart_esp32_rx();
}

void Communication_DebugInit(UART_HandleTypeDef *huart)
{
    debug_uart = huart;
    /* 先发一条启动消息确认 UART1 TX 硬件通路 */
    HAL_UART_Transmit(huart, (uint8_t *)"\r\n=== STM32 UART1 Debug Init OK ===\r\n", 38, 100);
    HAL_UARTEx_ReceiveToIdle_IT(huart, debug_rx_buf, DEBUG_RX_BUF_SIZE);
}

void Communication_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* UART1: 来自串口调试助手的数据 → 转发到 ESP32 */
    if (huart->Instance == USART1)
    {
        if (Size > 0)
            Communication_SendBytes(debug_rx_buf, Size);
        HAL_UARTEx_ReceiveToIdle_IT(huart, debug_rx_buf, DEBUG_RX_BUF_SIZE);
        return;
    }

    /* USART6: 来自 ESP32 的遥控帧 */
    if (huart->Instance != USART6 || Size == 0) {
        restart_esp32_rx();
        return;
    }

    /* 诊断记录 */
    diag_usart6_rx_count++;
    diag_last_rx_size = Size;
    uint16_t snap_len = Size < 16 ? Size : 16;
    memcpy((void *)diag_rx_snapshot, rx_it_buf, snap_len);

#if ESP32_LINK_BRIDGE_TEST
    if (debug_uart) {
        HAL_UART_Transmit(debug_uart, (uint8_t *)"[USART6<-] ", 11, 20);
        HAL_UART_Transmit(debug_uart, rx_it_buf, Size, 100);
        HAL_UART_Transmit(debug_uart, (uint8_t *)"\r\n", 2, 20);
    }
#endif

    handle_hello(rx_it_buf, Size);
    handle_motor_debug_command(rx_it_buf, Size);
    int parsed_len = parse_frame(rx_it_buf, Size);
    send_esp32_rx_echo(Size, parsed_len > 0);
    restart_esp32_rx();
}

int Communication_IsHandshakeDone(void)
{
    return handshake_done != 0;
}

int Communication_IsLinkAlive(void)
{
    return handshake_done && (HAL_GetTick() - last_valid_packet_tick) < ESP32_WATCHDOG_TIMEOUT_MS;
}

void Communication_ResetWatchdog(void)
{
    last_valid_packet_tick = HAL_GetTick();
}

void Communication_Task(void)
{
    if (!handshake_done && esp32_uart &&
        (HAL_GetTick() - last_hello_tick) >= 500) {
        last_hello_tick = HAL_GetTick();
        Communication_SendBytes((const uint8_t *)stm32_ack, sizeof(stm32_ack) - 1);
    }

    if (handshake_done && !Communication_IsLinkAlive()) {
        Communication_SetSafeRCData();
    }
}

void Communication_NotifyTxComplete(void)
{
    tx_it_busy = 0;
}

/* ---- DMA 发送到 ESP32（非阻塞，ISR 中可安全调用） ---- */
void Communication_SendByte(uint8_t byte)
{
    if (!esp32_uart || tx_it_busy)
        return;
    tx_it_buf[0] = byte;
    if (HAL_UART_Transmit_IT(esp32_uart, tx_it_buf, 1) == HAL_OK) {
        tx_it_busy = 1;
    }
}

void Communication_SendBytes(const uint8_t *data, uint16_t len)
{
    if (!esp32_uart || tx_it_busy || len > TX_IT_BUF_SIZE)
        return;
    memcpy(tx_it_buf, data, len);
    if (HAL_UART_Transmit_IT(esp32_uart, tx_it_buf, len) == HAL_OK) {
        tx_it_busy = 1;
    }
}

void Communication_SendString(const char *str)
{
    if (!str)
        return;
    Communication_SendBytes((const uint8_t *)str, strlen(str));
}

void Communication_SendMotorAngles(void)
{
    float angles[8];
    uint8_t valid[8];
    Leg_Control_GetAngles(angles, valid);

    char buf[192];
    int len = snprintf(buf, sizeof(buf),
        "MOTOR_ANGLES %.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f %u,%u,%u,%u,%u,%u,%u,%u\n",
        angles[0], angles[1], angles[2], angles[3],
        angles[4], angles[5], angles[6], angles[7],
        valid[0], valid[1], valid[2], valid[3],
        valid[4], valid[5], valid[6], valid[7]);

    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}

/* ---- 诊断输出：打印 USART6 接收状态（阻塞式，仅供调试用） ---- */
void Communication_SendMotorStatus(void)
{
    uint8_t motor_online[8];
    uint8_t leg_online[4];
    Leg_Control_GetOnline(motor_online, leg_online);

    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "MOTOR_STATUS %u,%u,%u,%u,%u,%u,%u,%u %u,%u,%u,%u\n",
        motor_online[0], motor_online[1], motor_online[2], motor_online[3],
        motor_online[4], motor_online[5], motor_online[6], motor_online[7],
        leg_online[0], leg_online[1], leg_online[2], leg_online[3]);

    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}

void Communication_DumpDiag(void)
{
    if (!debug_uart)
        return;

    char buf[160];
    int len = snprintf(buf, sizeof(buf),
        "[DIAG] USART6 RX cnt=%lu last_size=%u snapshot: "
        "%02X %02X %02X %02X %02X %02X %02X %02X "
        "%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        diag_usart6_rx_count,
        (unsigned int)diag_last_rx_size,
        diag_rx_snapshot[0], diag_rx_snapshot[1],
        diag_rx_snapshot[2], diag_rx_snapshot[3],
        diag_rx_snapshot[4], diag_rx_snapshot[5],
        diag_rx_snapshot[6], diag_rx_snapshot[7],
        diag_rx_snapshot[8], diag_rx_snapshot[9],
        diag_rx_snapshot[10], diag_rx_snapshot[11],
        diag_rx_snapshot[12], diag_rx_snapshot[13],
        diag_rx_snapshot[14], diag_rx_snapshot[15]);
    if (len > 0)
        HAL_UART_Transmit(debug_uart, (uint8_t *)buf, len, 100);
}

/* ---- 向 UART1 打印遥控数据（主循环调用，非 ISR 上下文） ---- */
void Communication_PrintRCData(void)
{
    if (!debug_uart || !rc_data_updated)
        return;

    __disable_irq();
    rc_data_updated = 0;
    RC_DataTypeDef tmp = rc_data;
    __enable_irq();

    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "RC: ch0=%+5d ch1=%+5d ch2=%+5d ch3=%+5d s1=%d s2=%d\r\n",
        tmp.ch0, tmp.ch1, tmp.ch2, tmp.ch3, tmp.s1, tmp.s2);
    if (len > 0)
        HAL_UART_Transmit(debug_uart, (uint8_t *)buf, len, 100);
}
