#include "communication.h"
#include <string.h>
#include <stdio.h>
#include "crc_ccitt.h"
#include "Leg_Control.h"

extern RC_DataTypeDef rc_data;
extern volatile uint32_t last_valid_packet_tick;

#define RX_BUF_SIZE 128
#define TX_IT_BUF_SIZE 256
#define RX_SNAPSHOT_SIZE 16

typedef struct
{
    UART_HandleTypeDef *uart;                  /* 连接 ESP32 的 USART6 句柄。 */
    uint8_t rx_buf[RX_BUF_SIZE];               /* 接收缓冲区，承载遥控帧和文本指令。 */
    uint8_t tx_buf[TX_IT_BUF_SIZE];            /* ESP32 回复共用的中断发送缓冲区。 */
    volatile uint8_t tx_busy;                  /* HAL_UART_Transmit_IT 发送期间为非 0。 */
    volatile uint8_t handshake_done;           /* 收到 ESP32 hello 或有效遥控帧后置位。 */
    uint32_t last_hello_tick;                  /* 握手完成前，ACK 重试的上次时间戳。 */
#if ESP32_LINK_ECHO_TEST
    uint32_t last_echo_tick;                   /* 回显调试的上次时间戳，用于限频。 */
#endif
    volatile uint32_t rx_count;                /* USART6 接收回调触发次数。 */
    volatile uint16_t last_rx_size;            /* 最近一次接收事件的字节数。 */
    volatile uint8_t rx_snapshot[RX_SNAPSHOT_SIZE]; /* 最近一次接收数据的前几字节快照。 */
    volatile uint16_t last_calc_crc;           /* 最近一次候选遥控帧计算得到的 CRC。 */
    volatile uint16_t last_recv_crc;           /* 最近一次候选遥控帧接收到的 CRC。 */
} Communication_ContextTypeDef;

static Communication_ContextTypeDef comm_ctx = {0};

static const char esp32_hello[] = "ESP32_HELLO";
static const char stm32_ack[] = "STM32_ACK\n";
static const char motor_set_mrad_cmd[] = "MOTOR_SET_MRAD";
static const char motor_set_cmd[] = "MOTOR_SET";
static const char motor_rescan_cmd[] = "MOTOR_RESCAN";
static const char motor_stop_all_cmd[] = "MOTOR_STOP_ALL";
static const char motor_hold_current_cmd[] = "MOTOR_HOLD_CURRENT";
static const char leg_nudge_mm_cmd[] = "LEG_NUDGE_MM";
static const char leg_trace_cmd[] = "LEG_TRACE";
static const char leg_snapshot_cmd[] = "LEG_SNAPSHOT";
static const char leg_all_micro_cmd[] = "LEG_ALL_MICRO";
static const char leg_prep_pose_cmd[] = "LEG_PREP_POSE";
static const char leg_stand_step_cmd[] = "LEG_STAND_STEP";
static const char leg_touch_step_cmd[] = "LEG_TOUCH_STEP";
static const char leg_loaded_step_cmd[] = "LEG_LOADED_STEP";

#define RC_RAW_MIN       0
#define RC_RAW_MAX       2047
#define RC_CENTER        1024
#define RC_DEADZONE      363


static void restart_esp32_rx(void)
{
    if (comm_ctx.uart) {
        HAL_UARTEx_ReceiveToIdle_IT(comm_ctx.uart, comm_ctx.rx_buf, RX_BUF_SIZE);
    }
}

static void send_esp32_rx_echo(uint16_t size, int parsed_frame)
{
#if ESP32_LINK_ECHO_TEST
    uint32_t now = HAL_GetTick();
    if (now - comm_ctx.last_echo_tick < 500) {
        return;
    }
    comm_ctx.last_echo_tick = now;

    char buf[80];
    int len;
    if (parsed_frame) {
        len = snprintf(buf, sizeof(buf),
                       "STM32_RX cnt=%lu size=%u frame=1\r\n",
                       comm_ctx.rx_count,
                       (unsigned int)size);
    } else {
        len = snprintf(buf, sizeof(buf),
                       "STM32_RX cnt=%lu size=%u frame=0 head=%02X%02X tail=%02X crc=%04X/%04X\r\n",
                       comm_ctx.rx_count,
                       (unsigned int)size,
                       comm_ctx.rx_snapshot[0],
                       comm_ctx.rx_snapshot[1],
                       size > 0 ? comm_ctx.rx_snapshot[(size < RX_SNAPSHOT_SIZE ? size : RX_SNAPSHOT_SIZE) - 1] : 0,
                       comm_ctx.last_recv_crc,
                       comm_ctx.last_calc_crc);
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
            comm_ctx.handshake_done = 1;
            comm_ctx.last_hello_tick = HAL_GetTick();
            Communication_SendBytes((const uint8_t *)stm32_ack, sizeof(stm32_ack) - 1);
            return;
        }
    }
}

static const char *skip_spaces(const char *p)
{
    while (*p == ' ' || *p == '\t')
        p++;
    return p;
}

static int parse_int_value(const char **text, int *value)
{
    const char *p = skip_spaces(*text);
    int sign = 1;
    int result = 0;
    int digits = 0;

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (*p >= '0' && *p <= '9') {
        result = result * 10 + (*p - '0');
        p++;
        digits++;
    }

    if (digits == 0)
        return 0;

    *value = sign * result;
    *text = p;
    return 1;
}

static int parse_decimal_value(const char **text, float *value)
{
    const char *p = skip_spaces(*text);
    int sign = 1;
    int32_t whole = 0;
    int32_t frac = 0;
    int32_t scale = 1;
    int digits = 0;

    if (*p == '-') {
        sign = -1;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while (*p >= '0' && *p <= '9') {
        whole = whole * 10 + (*p - '0');
        p++;
        digits++;
    }

    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            if (scale < 1000000) {
                frac = frac * 10 + (*p - '0');
                scale *= 10;
            }
            p++;
            digits++;
        }
    }

    if (digits == 0)
        return 0;

    *value = (float)sign * ((float)whole + ((float)frac / (float)scale));
    *text = p;
    return 1;
}

static int parse_motor_set_command(const char *cmd, int *motor, float *angle)
{
    const char prefix[] = "MOTOR_SET";
    const char *p = cmd;

    if (memcmp(p, prefix, sizeof(prefix) - 1) != 0)
        return 0;

    p += sizeof(prefix) - 1;
    if (*p != ' ' && *p != '\t')
        return 0;

    if (!parse_int_value(&p, motor))
        return 0;
    if (!parse_decimal_value(&p, angle))
        return 0;

    p = skip_spaces(p);
    return *p == '\0';
}

static int parse_motor_set_mrad_command(const char *cmd, int *motor, float *angle)
{
    const char prefix[] = "MOTOR_SET_MRAD";
    const char *p = cmd;
    int angle_mrad = 0;

    if (memcmp(p, prefix, sizeof(prefix) - 1) != 0)
        return 0;

    p += sizeof(prefix) - 1;
    if (*p != ' ' && *p != '\t')
        return 0;

    if (!parse_int_value(&p, motor))
        return 0;
    if (!parse_int_value(&p, &angle_mrad))
        return 0;

    p = skip_spaces(p);
    if (*p != '\0')
        return 0;

    *angle = (float)angle_mrad / 1000.0f;
    return 1;
}

static void handle_motor_set_text(const char *cmd_buf)
{
    int motor = -1;
    float angle = 0.0f;
    int parsed = 0;
    int accepted = 0;
    char log_buf[96];

    parsed = parse_motor_set_mrad_command(cmd_buf, &motor, &angle) ||
             parse_motor_set_command(cmd_buf, &motor, &angle);
    if (!parsed) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "MOTOR_SET_RX parse_fail cmd=%s\r\n", cmd_buf);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
        return;
    }

    if (motor >= 0 && motor < 8) {
        accepted = Leg_Control_SetDebugAngle((uint8_t)motor, angle);
    }

    if (!accepted) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "MOTOR_SET_RX rejected motor=%d cmd=%s\r\n",
                               motor, cmd_buf);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
    }
}

static int parse_leg_nudge_mm_command(const char *cmd, int *leg, float *dx_mm, float *dy_mm)
{
    const char *p = cmd;
    int dx_int = 0;
    int dy_int = 0;

    if (memcmp(p, leg_nudge_mm_cmd, sizeof(leg_nudge_mm_cmd) - 1) != 0)
        return 0;

    p += sizeof(leg_nudge_mm_cmd) - 1;
    if (*p != ' ' && *p != '\t')
        return 0;

    if (!parse_int_value(&p, leg))
        return 0;
    if (!parse_int_value(&p, &dx_int))
        return 0;
    if (!parse_int_value(&p, &dy_int))
        return 0;

    p = skip_spaces(p);
    if (*p != '\0')
        return 0;

    *dx_mm = (float)dx_int;
    *dy_mm = (float)dy_int;
    return 1;
}

static void handle_leg_nudge_text(const char *cmd_buf)
{
    int leg = -1;
    float dx_mm = 0.0f;
    float dy_mm = 0.0f;
    int accepted = 0;
    char log_buf[96];

    if (!parse_leg_nudge_mm_command(cmd_buf, &leg, &dx_mm, &dy_mm)) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "LEG_NUDGE_RX parse_fail cmd=%s\r\n", cmd_buf);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
        return;
    }

    if (leg >= 0 && leg < 4) {
        accepted = Leg_Control_SetDebugFootOffset((uint8_t)leg, dx_mm, dy_mm);
    }

    if (!accepted) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "LEG_NUDGE_RX rejected leg=%d dx=%ld dy=%ld\r\n",
                               leg, (long)dx_mm, (long)dy_mm);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
    }
}

static int parse_leg_trace_command(const char *cmd, int *leg)
{
    const char *p = cmd;

    if (memcmp(p, leg_trace_cmd, sizeof(leg_trace_cmd) - 1) != 0)
        return 0;

    p += sizeof(leg_trace_cmd) - 1;
    if (*p != ' ' && *p != '\t')
        return 0;

    if (!parse_int_value(&p, leg))
        return 0;

    p = skip_spaces(p);
    return *p == '\0';
}

static void handle_leg_trace_text(const char *cmd_buf)
{
    int leg = -1;
    int accepted = 0;
    char log_buf[96];

    if (!parse_leg_trace_command(cmd_buf, &leg)) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "LEG_TRACE_RX parse_fail cmd=%s\r\n", cmd_buf);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
        return;
    }

    if (leg >= 0 && leg < 4) {
        accepted = Leg_Control_StartDebugTrace((uint8_t)leg);
    }

    if (!accepted) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "LEG_TRACE_RX rejected leg=%d\r\n", leg);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
    }
}

static void handle_motor_debug_command(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(motor_set_cmd) - 1 &&
        len < sizeof(motor_set_mrad_cmd) - 1 &&
        len < sizeof(motor_rescan_cmd) - 1 &&
        len < sizeof(motor_stop_all_cmd) - 1 &&
        len < sizeof(motor_hold_current_cmd) - 1 &&
        len < sizeof(leg_nudge_mm_cmd) - 1 &&
        len < sizeof(leg_trace_cmd) - 1 &&
        len < sizeof(leg_snapshot_cmd) - 1 &&
        len < sizeof(leg_all_micro_cmd) - 1 &&
        len < sizeof(leg_prep_pose_cmd) - 1 &&
        len < sizeof(leg_stand_step_cmd) - 1 &&
        len < sizeof(leg_touch_step_cmd) - 1 &&
        len < sizeof(leg_loaded_step_cmd) - 1)
        return;

    for (uint16_t i = 0; i + sizeof(motor_stop_all_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_stop_all_cmd, sizeof(motor_stop_all_cmd) - 1) == 0) {
            Leg_Control_StopAllDebugTargets(0);
            Communication_SendString("MOTOR_STOP_ALL ok\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_hold_current_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_hold_current_cmd, sizeof(motor_hold_current_cmd) - 1) == 0) {
            if (!Leg_Control_HoldCurrentPosition()) {
                Communication_SendString("MOTOR_HOLD_CURRENT rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_rescan_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_rescan_cmd, sizeof(motor_rescan_cmd) - 1) == 0) {
            Leg_Control_RequestHandshake();
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_snapshot_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_snapshot_cmd, sizeof(leg_snapshot_cmd) - 1) == 0) {
            Leg_Control_LogFootSnapshot();
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_all_micro_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_all_micro_cmd, sizeof(leg_all_micro_cmd) - 1) == 0) {
            if (!Leg_Control_StartAllMicroTest()) {
                Communication_SendString("LEG_ALL_MICRO_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_prep_pose_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_prep_pose_cmd, sizeof(leg_prep_pose_cmd) - 1) == 0) {
            if (!Leg_Control_StartPrepPoseTest()) {
                Communication_SendString("LEG_PREP_POSE_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_stand_step_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_stand_step_cmd, sizeof(leg_stand_step_cmd) - 1) == 0) {
            if (!Leg_Control_StartStandStepTest()) {
                Communication_SendString("LEG_STAND_STEP_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_touch_step_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_touch_step_cmd, sizeof(leg_touch_step_cmd) - 1) == 0) {
            if (!Leg_Control_StartTouchStepTest()) {
                Communication_SendString("LEG_TOUCH_STEP_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_loaded_step_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_loaded_step_cmd, sizeof(leg_loaded_step_cmd) - 1) == 0) {
            if (!Leg_Control_StartLoadedStepTest()) {
                Communication_SendString("LEG_LOADED_STEP_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_nudge_mm_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_nudge_mm_cmd, sizeof(leg_nudge_mm_cmd) - 1) == 0) {
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf))
                copy_len = sizeof(cmd_buf) - 1;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            for (uint16_t j = 0; j < copy_len; j++) {
                if (cmd_buf[j] == '\r' || cmd_buf[j] == '\n') {
                    cmd_buf[j] = '\0';
                    break;
                }
            }

            handle_leg_nudge_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_trace_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_trace_cmd, sizeof(leg_trace_cmd) - 1) == 0) {
            char cmd_buf[32];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf))
                copy_len = sizeof(cmd_buf) - 1;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            for (uint16_t j = 0; j < copy_len; j++) {
                if (cmd_buf[j] == '\r' || cmd_buf[j] == '\n') {
                    cmd_buf[j] = '\0';
                    break;
                }
            }

            handle_leg_trace_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_set_mrad_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_set_mrad_cmd, sizeof(motor_set_mrad_cmd) - 1) == 0) {
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf))
                copy_len = sizeof(cmd_buf) - 1;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            for (uint16_t j = 0; j < copy_len; j++) {
                if (cmd_buf[j] == '\r' || cmd_buf[j] == '\n') {
                    cmd_buf[j] = '\0';
                    break;
                }
            }

            handle_motor_set_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_set_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_set_cmd, sizeof(motor_set_cmd) - 1) == 0) {
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf))
                copy_len = sizeof(cmd_buf) - 1;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            for (uint16_t j = 0; j < copy_len; j++) {
                if (cmd_buf[j] == '\r' || cmd_buf[j] == '\n') {
                    cmd_buf[j] = '\0';
                    break;
                }
            }

            handle_motor_set_text(cmd_buf);
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
        comm_ctx.last_calc_crc = calc_crc;
        comm_ctx.last_recv_crc = recv_crc;
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
        __enable_irq();
        comm_ctx.handshake_done = 1;
        last_valid_packet_tick = HAL_GetTick();
        return i + ESP32_FRAME_LEN;
    }
    return 0;
}

void Communication_Init(UART_HandleTypeDef *huart)
{
    memset(&comm_ctx, 0, sizeof(comm_ctx));
    comm_ctx.uart = huart;
    comm_ctx.last_hello_tick = HAL_GetTick();

    Communication_SetSafeRCData();
    last_valid_packet_tick = 0;

    restart_esp32_rx();
}

void Communication_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* USART6 承载来自 ESP32 的遥控帧和文本调试指令。 */
    if (huart->Instance != USART6 || Size == 0) {
        restart_esp32_rx();
        return;
    }

    /* 保留一小段接收快照，供可选回显调试使用。 */
    comm_ctx.rx_count++;
    comm_ctx.last_rx_size = Size;
    uint16_t snap_len = Size < RX_SNAPSHOT_SIZE ? Size : RX_SNAPSHOT_SIZE;
    memcpy((void *)comm_ctx.rx_snapshot, comm_ctx.rx_buf, snap_len);

    handle_hello(comm_ctx.rx_buf, Size);
    handle_motor_debug_command(comm_ctx.rx_buf, Size);
    int parsed_len = parse_frame(comm_ctx.rx_buf, Size);
    send_esp32_rx_echo(Size, parsed_len > 0);
    restart_esp32_rx();
}

int Communication_IsHandshakeDone(void)
{
    return comm_ctx.handshake_done != 0;
}

int Communication_IsLinkAlive(void)
{
    return comm_ctx.handshake_done && (HAL_GetTick() - last_valid_packet_tick) < ESP32_WATCHDOG_TIMEOUT_MS;
}

void Communication_ResetWatchdog(void)
{
    last_valid_packet_tick = HAL_GetTick();
}

void Communication_Task(void)
{
    if (!comm_ctx.handshake_done && comm_ctx.uart &&
        (HAL_GetTick() - comm_ctx.last_hello_tick) >= 500) {
        comm_ctx.last_hello_tick = HAL_GetTick();
        Communication_SendBytes((const uint8_t *)stm32_ack, sizeof(stm32_ack) - 1);
    }

    if (comm_ctx.handshake_done && !Communication_IsLinkAlive()) {
        Communication_SetSafeRCData();
    }
}

void Communication_NotifyTxComplete(void)
{
    comm_ctx.tx_busy = 0;
}

/* 中断发送共用 comm_ctx.tx_buf，忙碌时直接丢弃本次发送。 */
void Communication_SendByte(uint8_t byte)
{
    if (!comm_ctx.uart || comm_ctx.tx_busy)
        return;
    comm_ctx.tx_buf[0] = byte;
    if (HAL_UART_Transmit_IT(comm_ctx.uart, comm_ctx.tx_buf, 1) == HAL_OK) {
        comm_ctx.tx_busy = 1;
    }
}

void Communication_SendBytes(const uint8_t *data, uint16_t len)
{
    if (!comm_ctx.uart || comm_ctx.tx_busy || len > TX_IT_BUF_SIZE)
        return;
    memcpy(comm_ctx.tx_buf, data, len);
    if (HAL_UART_Transmit_IT(comm_ctx.uart, comm_ctx.tx_buf, len) == HAL_OK) {
        comm_ctx.tx_busy = 1;
    }
}

void Communication_SendString(const char *str)
{
    if (!str)
        return;
    Communication_SendBytes((const uint8_t *)str, strlen(str));
}

static int append_fixed4(char *buf, size_t size, int pos, float value)
{
    if (pos < 0 || (size_t)pos >= size)
        return -1;

    int32_t scaled;
    if (value >= 0.0f)
        scaled = (int32_t)(value * 10000.0f + 0.5f);
    else
        scaled = (int32_t)(value * 10000.0f - 0.5f);

    const char *sign = "";
    if (scaled < 0) {
        sign = "-";
        scaled = -scaled;
    }

    int written = snprintf(&buf[pos], size - (size_t)pos, "%s%ld.%04ld",
                           sign,
                           (long)(scaled / 10000),
                           (long)(scaled % 10000));
    if (written < 0 || written >= (int)(size - (size_t)pos))
        return -1;

    return pos + written;
}

void Communication_SendMotorAngles(void)
{
    float angles[8];
    uint8_t valid[8];
    Leg_Control_GetAngles(angles, valid);

    char buf[192];
    int len = snprintf(buf, sizeof(buf), "MOTOR_ANGLES ");
    for (uint8_t i = 0; i < 8 && len > 0; i++) {
        len = append_fixed4(buf, sizeof(buf), len, angles[i]);
        if (len > 0) {
            int written = snprintf(&buf[len], sizeof(buf) - (size_t)len,
                                   "%c", (i == 7) ? ' ' : ',');
            if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
                len = -1;
            else
                len += written;
        }
    }

    if (len > 0) {
        int written = snprintf(&buf[len], sizeof(buf) - (size_t)len,
        "%u,%u,%u,%u,%u,%u,%u,%u\n",
        valid[0], valid[1], valid[2], valid[3],
        valid[4], valid[5], valid[6], valid[7]);
        if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
            len = -1;
        else
            len += written;
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}

static int append_motor_zero(char *buf, size_t size, int len)
{
    float zero_error[8];
    uint8_t zero_ok[8];
    uint8_t all_zero_ok = 0;
    Leg_Control_GetZeroCheck(zero_error, zero_ok, &all_zero_ok);

    if (len < 0 || (size_t)len >= size)
        return -1;

    int written = snprintf(&buf[len], size - (size_t)len, "MOTOR_ZERO ");
    if (written < 0 || written >= (int)(size - (size_t)len))
        return -1;
    len += written;

    for (uint8_t i = 0; i < 8 && len > 0; i++) {
        len = append_fixed4(buf, size, len, zero_error[i]);
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len,
                                   "%c", (i == 7) ? ' ' : ',');
            if (written < 0 || written >= (int)(size - (size_t)len))
                len = -1;
            else
                len += written;
        }
    }

    if (len > 0) {
        written = snprintf(&buf[len], size - (size_t)len,
            "%u,%u,%u,%u,%u,%u,%u,%u %u ",
            zero_ok[0], zero_ok[1], zero_ok[2], zero_ok[3],
            zero_ok[4], zero_ok[5], zero_ok[6], zero_ok[7],
            all_zero_ok);
        if (written < 0 || written >= (int)(size - (size_t)len))
            len = -1;
        else
            len += written;
    }

    if (len > 0) {
        len = append_fixed4(buf, size, len, Leg_Control_GetZeroThreshold());
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len, "\n");
            if (written < 0 || written >= (int)(size - (size_t)len))
                len = -1;
            else
                len += written;
        }
    }

    return len;
}

/* 向 ESP32 网页端发送紧凑的电机握手与目标状态。 */
void Communication_SendMotorStatus(void)
{
    uint8_t motor_online[8];
    uint8_t leg_online[4];
    uint8_t motor_error[8];
    uint8_t target_active[8];
    uint8_t target_result[8];
    Leg_Control_GetOnline(motor_online, leg_online);
    Leg_Control_GetHandshakeErrors(motor_error);
    Leg_Control_GetTargetStates(target_active, target_result);

    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "MOTOR_STATUS %u,%u,%u,%u,%u,%u,%u,%u %u,%u,%u,%u "
        "%u,%u,%u,%u,%u,%u,%u,%u %u,%u,%u,%u,%u,%u,%u,%u %u,%u,%u,%u,%u,%u,%u,%u\n",
        motor_online[0], motor_online[1], motor_online[2], motor_online[3],
        motor_online[4], motor_online[5], motor_online[6], motor_online[7],
        leg_online[0], leg_online[1], leg_online[2], leg_online[3],
        motor_error[0], motor_error[1], motor_error[2], motor_error[3],
        motor_error[4], motor_error[5], motor_error[6], motor_error[7],
        target_active[0], target_active[1], target_active[2], target_active[3],
        target_active[4], target_active[5], target_active[6], target_active[7],
        target_result[0], target_result[1], target_result[2], target_result[3],
        target_result[4], target_result[5], target_result[6], target_result[7]);

    if (len > 0 && len < (int)sizeof(buf)) {
        len = append_motor_zero(buf, sizeof(buf), len);
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}
