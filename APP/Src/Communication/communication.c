#include "communication.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "crc_ccitt.h"
#include "Leg_Control.h"
#include "Leg_Gait.h"
#include "Motor_Transport.h"

extern RC_DataTypeDef rc_data;
extern volatile uint32_t last_valid_packet_tick;

#define RX_BUF_SIZE 128
#define TX_IT_BUF_SIZE 896
#define RX_SNAPSHOT_SIZE 16
#define TEXT_COMMAND_BUF_SIZE 128
#define RX_CHUNK_QUEUE_DEPTH 8U

typedef struct
{
    UART_HandleTypeDef *uart;                  /* 连接 ESP32 的 USART6 句柄。 */
    uint8_t rx_buf[RX_BUF_SIZE];               /* 当前 HAL Receive-to-Idle 缓冲。 */
    volatile uint8_t rx_rearm_pending;         /* 挂接失败时由前台任务重试。 */
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
/* A control state reply is operator-facing telemetry.  Unlike periodic logs,
 * retain one pending reply when USART6 is occupied by angle/status telemetry. */
static volatile uint8_t motor_control_status_pending = 0U;
static volatile uint8_t leg_trajectory_status_pending = 0U;
/* Receive-to-idle can split one UART text command across callbacks.  Keep a
 * foreground line accumulator so command recognition never depends on the
 * callback boundary chosen by the UART idle detector. */
static char text_command_buf[TEXT_COMMAND_BUF_SIZE];
static uint16_t text_command_len = 0U;
typedef struct
{
    uint16_t len;
    uint8_t data[RX_BUF_SIZE];
} Communication_RxChunkTypeDef;
static Communication_RxChunkTypeDef rx_chunk_queue[RX_CHUNK_QUEUE_DEPTH];
static volatile uint8_t rx_chunk_read = 0U;
static volatile uint8_t rx_chunk_write = 0U;
static volatile uint32_t rx_chunk_overflow_count = 0U;

static const char esp32_hello[] = "ESP32_HELLO";
static const char stm32_ack[] = "STM32_ACK\n";
static void handle_pid_control_text(const char *cmd_buf);
static const char motor_set_mrad_cmd[] = "MOTOR_SET_MRAD";
static const char motor_set_cmd[] = "MOTOR_SET";
static const char motor_rescan_cmd[] = "MOTOR_RESCAN";
static const char motor_stop_all_cmd[] = "MOTOR_STOP_ALL";
static const char motor_hold_current_cmd[] = "MOTOR_HOLD_CURRENT";
static const char motor_zero_all_arm_cmd[] = "MOTOR_ZERO_ALL_ARM";
static const char motor_zero_all_run_cmd[] = "MOTOR_ZERO_ALL_RUN";
static const char motor_group_arm_cmd[] = "MOTOR_GROUP_ARM";
static const char motor_group_run_cmd[] = "MOTOR_GROUP_RUN";
static const char motor_stand_arm_cmd[] = "MOTOR_STAND_ARM";
static const char pid_arm_cmd[] = "PID_ARM";
static const char pid_plan_cmd[] = "PID_PLAN";
static const char pid_hold_dryrun_cmd[] = "PID_HOLD_DRYRUN";
static const char pid_hold_active_cmd[] = "PID_HOLD_ACTIVE";
static const char pid_traj_dryrun_cmd[] = "PID_TRAJ_DRYRUN";
static const char pid_traj_active_cmd[] = "PID_TRAJ_ACTIVE";
static const char leg_traj_arm_rf_cmd[] = "LEG_TRAJ_ARM_RF";
static const char leg_traj_dryrun_cmd[] = "LEG_TRAJ_DRYRUN";
static const char leg_traj_active_cmd[] = "LEG_TRAJ_ACTIVE";
static const char leg_traj_hold_cmd[] = "LEG_TRAJ_HOLD";
static const char leg_nudge_mm_cmd[] = "LEG_NUDGE_MM";
static const char leg_trace_cmd[] = "LEG_TRACE";
static const char leg_snapshot_cmd[] = "LEG_SNAPSHOT";
static const char leg_all_micro_cmd[] = "LEG_ALL_MICRO";
static const char leg_prep_pose_cmd[] = "LEG_PREP_POSE";
static const char leg_sine_cmd[] = "LEG_SINE";
static const char leg_trot_cmd[] = "LEG_TROT";
static const char leg_stand_step_cmd[] = "LEG_STAND_STEP";
static const char leg_touch_step_cmd[] = "LEG_TOUCH_STEP";
static const char leg_loaded_step_cmd[] = "LEG_LOADED_STEP";

#define RC_RAW_MIN       0
#define RC_RAW_MAX       2047
#define RC_CENTER        1024
#define RC_DEADZONE      363


static HAL_StatusTypeDef arm_esp32_rx(void)
{
    if (!comm_ctx.uart) return HAL_ERROR;
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_IT(
        comm_ctx.uart, comm_ctx.rx_buf, RX_BUF_SIZE);
    if (status == HAL_OK) {
        comm_ctx.rx_rearm_pending = 0U;
    } else {
        comm_ctx.rx_rearm_pending = 1U;
    }
    return status;
}

static void restart_esp32_rx(void)
{
    (void)arm_esp32_rx();
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
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
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
    Communication_SendMotorControlStatus();
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
        accepted = Leg_Gait_StartDebugTrace((uint8_t)leg);
    }

    if (!accepted) {
        int log_len = snprintf(log_buf, sizeof(log_buf),
                               "LEG_TRACE_RX rejected leg=%d\r\n", leg);
        if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
            Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
        }
    }
}

static int parse_leg_sine_command(const char *cmd, int *leg, float *amp_mm, float *freq_hz)
{
  const char *p = cmd;

  if (memcmp(p, leg_sine_cmd, sizeof(leg_sine_cmd) - 1) != 0)
    return 0;

  p += sizeof(leg_sine_cmd) - 1;
  if (*p != ' ' && *p != '\t')
    return 0;

  if (!parse_int_value(&p, leg))
    return 0;
  if (!parse_decimal_value(&p, amp_mm))
    return 0;
  if (!parse_decimal_value(&p, freq_hz))
    return 0;

  p = skip_spaces(p);
  return *p == '\0';
}

static void handle_leg_sine_text(const char *cmd_buf)
{
  int leg = -1;
  float amp_mm = 0.0f;
  float freq_hz = 0.0f;
  int accepted = 0;
  char log_buf[96];

  if (!parse_leg_sine_command(cmd_buf, &leg, &amp_mm, &freq_hz)) {
    int log_len = snprintf(log_buf, sizeof(log_buf),
                           "LEG_SINE_RX parse_fail cmd=%s\r\n", cmd_buf);
    if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
      Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
    }
    return;
  }

  if (leg >= 0 && leg < 4) {
    accepted = Leg_Gait_StartSineTest((uint8_t)leg, amp_mm, freq_hz);
  }

  if (!accepted) {
    int log_len = snprintf(log_buf, sizeof(log_buf),
                           "LEG_SINE_RX rejected leg=%d amp=", leg);
    if (log_len > 0 && log_len < (int)sizeof(log_buf)) {
      Communication_SendBytes((const uint8_t *)log_buf, (uint16_t)log_len);
    }
  }
}

static int parse_motor_group_arm_command(const char *cmd, float *offset)
{
    const char *p = cmd;
    if (memcmp(p, motor_group_arm_cmd, sizeof(motor_group_arm_cmd) - 1) != 0)
        return 0;
    p += sizeof(motor_group_arm_cmd) - 1;
    if (*p != ' ' && *p != '\t') return 0;
    if (!parse_decimal_value(&p, offset)) return 0;
    p = skip_spaces(p);
    return *p == '\0';
}

static void handle_motor_group_arm_text(const char *cmd_buf)
{
    float offset = 0.0f;
    if (!parse_motor_group_arm_command(cmd_buf, &offset)) {
        Communication_SendString("MOTOR_GROUP_ARM parse_fail\r\n");
        return;
    }
    if (Leg_Control_ArmAllOffset(offset))
        Communication_SendString("MOTOR_GROUP_ARM ok\r\n");
    else
        Communication_SendString("MOTOR_GROUP_ARM rejected\r\n");
}

static void send_leg_trajectory_arm_audit(void)
{
    Motor_LegTrajectorySnapshot s;
    Leg_Control_GetRfLegTrajectorySnapshot(&s);
    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(
        buf, sizeof(buf),
        "LEG_TRAJ_ARM {\"ok\":%u,\"level\":%u,\"leg\":%u,"
        "\"idx\":[2,3],\"map\":[\"theta1_AB\",\"theta2_AD\"],"
        "\"arm\":[%ld,%ld],\"zero\":[%ld,%ld],\"dir\":[%d,%d],"
        "\"joint\":[%ld,%ld],\"ik\":[%ld,%ld],"
        "\"base\":[%ld,%ld],\"peak_y\":%ld,\"delta\":[%ld,%ld],"
        "\"temp\":[%d,%d],\"age\":[%lu,%lu],\"zero_ok\":[%u,%u],"
        "\"active\":%u}\n",
        (unsigned int)(s.mode == Motor_LegTrajectory_Armed),
        (unsigned int)s.profile.level,
        (unsigned int)MOTOR_LEG_TRAJECTORY_LEG_INDEX,
        (long)(s.arm_position[0] * 1000000.0f),
        (long)(s.arm_position[1] * 1000000.0f),
        (long)(s.zero_position[0] * 1000000.0f),
        (long)(s.zero_position[1] * 1000000.0f),
        (int)s.direction[0], (int)s.direction[1],
        (long)(s.arm_joint_position[0] * 1000000.0f),
        (long)(s.arm_joint_position[1] * 1000000.0f),
        (long)(s.base_joint_position[0] * 1000000.0f),
        (long)(s.base_joint_position[1] * 1000000.0f),
        (long)(s.base_foot.x * 1000.0f),
        (long)(s.base_foot.y * 1000.0f),
        (long)((s.base_foot.y - s.profile.lift_mm) *
               1000.0f),
        (long)(s.peak_target_delta[0] * 1000000.0f),
        (long)(s.peak_target_delta[1] * 1000000.0f),
        (int)s.arm_temperature_c[0], (int)s.arm_temperature_c[1],
        (unsigned long)s.arm_feedback_age_ms[0],
        (unsigned long)s.arm_feedback_age_ms[1],
        (unsigned int)s.arm_zero_checked[0],
        (unsigned int)s.arm_zero_checked[1],
        (unsigned int)MOTOR_LEG_TRAJECTORY_ACTIVE_ENABLED);
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

static void send_leg_trajectory_plan(uint8_t dry_run)
{
    Motor_LegTrajectorySnapshot s;
    Leg_Control_GetRfLegTrajectorySnapshot(&s);
    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(
        buf, sizeof(buf),
        "LEG_TRAJ_PLAN {\"level\":%u,\"dry\":%u,\"out\":%u,"
        "\"active\":%u,\"leg\":%u,\"idx\":[2,3],"
        "\"path\":\"vertical_cos\",\"lift\":%ld,\"period\":%lu,"
        "\"settle\":%lu,\"x0\":%ld,\"y0\":%ld,"
        "\"yp\":%ld,\"d\":[%ld,%ld],\"pkp\":%ld,\"pkd\":%ld,"
        "\"skp\":%ld,\"ski\":%ld,\"skd\":%ld,\"tmax\":%ld,"
        "\"vt\":%ld,\"vm\":%ld,\"xm\":%ld,\"tdm\":%ld,"
        "\"em\":%ld,\"dt_us\":[200,10000],\"fb_ms\":10,"
        "\"temp\":40,\"dur\":%lu,\"end\":\"%s\"}\n",
        (unsigned int)s.profile.level,
        (unsigned int)dry_run,
        (unsigned int)(dry_run != 0U ? 0U : 1U),
        (unsigned int)MOTOR_LEG_TRAJECTORY_ACTIVE_ENABLED,
        (unsigned int)MOTOR_LEG_TRAJECTORY_LEG_INDEX,
        (long)(s.profile.lift_mm * 1000.0f),
        (unsigned long)s.profile.period_ms,
        (unsigned long)s.profile.settle_ms,
        (long)(s.base_foot.x * 1000.0f),
        (long)(s.base_foot.y * 1000.0f),
        (long)((s.base_foot.y - s.profile.lift_mm) * 1000.0f),
        (long)(s.peak_target_delta[0] * 1000000.0f),
        (long)(s.peak_target_delta[1] * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_POSITION_KP * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_POSITION_KD * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_SPEED_KP * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_SPEED_KI * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_SPEED_KD * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_TORQUE_MAX_NM * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_TARGET_SPEED_MAX * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_ACTUAL_SPEED_MAX * 1000000.0f),
        (long)(s.profile.position_max * 1000000.0f),
        (long)(s.profile.target_delta_max * 1000000.0f),
        (long)(MOTOR_LEG_TRAJECTORY_TRACKING_ERROR_MAX * 1000000.0f),
        (unsigned long)s.profile.duration_ms,
        dry_run != 0U ? "zero_output" : "hold_arm");
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

static void handle_motor_debug_command(const uint8_t *data, uint16_t len)
{
    if (len < sizeof(motor_set_cmd) - 1 &&
        len < sizeof(motor_set_mrad_cmd) - 1 &&
        len < sizeof(motor_rescan_cmd) - 1 &&
        len < sizeof(motor_stop_all_cmd) - 1 &&
        len < sizeof(motor_hold_current_cmd) - 1 &&
        len < sizeof(motor_zero_all_arm_cmd) - 1 &&
        len < sizeof(motor_zero_all_run_cmd) - 1 &&
        len < sizeof(motor_group_arm_cmd) - 1 &&
        len < sizeof(motor_group_run_cmd) - 1 &&
        len < sizeof(motor_stand_arm_cmd) - 1 &&
        len < sizeof(pid_arm_cmd) - 1 &&
        len < sizeof(pid_plan_cmd) - 1 &&
        len < sizeof(pid_hold_dryrun_cmd) - 1 &&
        len < sizeof(pid_hold_active_cmd) - 1 &&
        len < sizeof(pid_traj_dryrun_cmd) - 1 &&
        len < sizeof(pid_traj_active_cmd) - 1 &&
        len < sizeof(leg_traj_arm_rf_cmd) - 1 &&
        len < sizeof(leg_traj_dryrun_cmd) - 1 &&
        len < sizeof(leg_traj_active_cmd) - 1 &&
        len < sizeof(leg_traj_hold_cmd) - 1 &&
        len < sizeof(leg_nudge_mm_cmd) - 1 &&
        len < sizeof(leg_trace_cmd) - 1 &&
        len < sizeof(leg_snapshot_cmd) - 1 &&
        len < sizeof(leg_all_micro_cmd) - 1 &&
        len < sizeof(leg_prep_pose_cmd) - 1 &&
        len < sizeof(leg_sine_cmd) - 1 &&
        len < sizeof(leg_trot_cmd) - 1 &&
        len < sizeof(leg_stand_step_cmd) - 1 &&
        len < sizeof(leg_touch_step_cmd) - 1 &&
        len < sizeof(leg_loaded_step_cmd) - 1)
        return;

    for (uint16_t i = 0;
         i + sizeof(leg_traj_arm_rf_cmd) - 1 <= len; ++i) {
        if (memcmp(&data[i], leg_traj_arm_rf_cmd,
                   sizeof(leg_traj_arm_rf_cmd) - 1) == 0) {
            if (i != 0U) continue;
            uint8_t level = MOTOR_LEG_TRAJECTORY_LEVEL_MIN;
            uint16_t pos = i + (uint16_t)(sizeof(leg_traj_arm_rf_cmd) - 1U);
            while (pos < len && (data[pos] == ' ' || data[pos] == '\t')) ++pos;
            if (pos < len) {
                if (data[pos] < '1' || data[pos] > '2') {
                    Communication_SendString("LEG_TRAJ_ARM parse_fail\r\n");
                    return;
                }
                level = (uint8_t)(data[pos++] - '0');
                while (pos < len && (data[pos] == ' ' || data[pos] == '\t'))
                    ++pos;
                if (pos != len) {
                    Communication_SendString("LEG_TRAJ_ARM parse_fail\r\n");
                    return;
                }
            }
            (void)Leg_Control_ArmRfLegTrajectory(level);
            send_leg_trajectory_arm_audit();
            Communication_SendLegTrajectoryStatus();
            return;
        }
    }

    for (uint16_t i = 0;
         i + sizeof(leg_traj_dryrun_cmd) - 1 <= len; ++i) {
        if (memcmp(&data[i], leg_traj_dryrun_cmd,
                   sizeof(leg_traj_dryrun_cmd) - 1) == 0) {
            if (Leg_Control_StartRfLegTrajectoryDryRun()) {
                send_leg_trajectory_plan(1U);
            } else {
                Communication_SendString("LEG_TRAJ_DRYRUN rejected\r\n");
            }
            Communication_SendLegTrajectoryStatus();
            return;
        }
    }

    for (uint16_t i = 0;
         i + sizeof(leg_traj_active_cmd) - 1 <= len; ++i) {
        if (memcmp(&data[i], leg_traj_active_cmd,
                   sizeof(leg_traj_active_cmd) - 1) == 0) {
            if (Leg_Control_StartRfLegTrajectoryActive()) {
                send_leg_trajectory_plan(0U);
            } else {
                Communication_SendString("LEG_TRAJ_ACTIVE rejected\r\n");
            }
            Communication_SendLegTrajectoryStatus();
            return;
        }
    }

    for (uint16_t i = 0;
         i + sizeof(leg_traj_hold_cmd) - 1 <= len; ++i) {
        if (memcmp(&data[i], leg_traj_hold_cmd,
                   sizeof(leg_traj_hold_cmd) - 1) == 0) {
            if (Leg_Control_HoldRfLegTrajectory())
                Communication_SendString("LEG_TRAJ_HOLD ok\r\n");
            else
                Communication_SendString("LEG_TRAJ_HOLD rejected\r\n");
            Communication_SendLegTrajectoryTelemetry();
            Communication_SendLegTrajectoryStatus();
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_stand_arm_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_stand_arm_cmd,
                   sizeof(motor_stand_arm_cmd) - 1) == 0) {
            if (Leg_Control_ArmStandPose())
                Communication_SendString("MOTOR_STAND_ARM ok\r\n");
            else
                Communication_SendString("MOTOR_STAND_ARM rejected\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_group_arm_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_group_arm_cmd,
                   sizeof(motor_group_arm_cmd) - 1) == 0) {
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_motor_group_arm_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_group_run_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_group_run_cmd,
                   sizeof(motor_group_run_cmd) - 1) == 0) {
            if (Leg_Control_StartAllZero())
                Communication_SendString("MOTOR_GROUP_RUN ok\r\n");
            else
                Communication_SendString("MOTOR_GROUP_RUN rejected\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(pid_hold_active_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], pid_hold_active_cmd,
                   sizeof(pid_hold_active_cmd) - 1) == 0) {
            char cmd_buf[40];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_pid_control_text(cmd_buf);
            return;
        }
    }
    for (uint16_t i = 0; i + sizeof(pid_traj_active_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], pid_traj_active_cmd,
                   sizeof(pid_traj_active_cmd) - 1) == 0) {
            char cmd_buf[40];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_pid_control_text(cmd_buf);
            return;
        }
    }
    for (uint16_t i = 0; i + sizeof(pid_hold_dryrun_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], pid_hold_dryrun_cmd,
                   sizeof(pid_hold_dryrun_cmd) - 1) == 0) {
            char cmd_buf[40];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_pid_control_text(cmd_buf);
            return;
        }
    }
    for (uint16_t i = 0; i + sizeof(pid_traj_dryrun_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], pid_traj_dryrun_cmd,
                   sizeof(pid_traj_dryrun_cmd) - 1) == 0) {
            char cmd_buf[40];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_pid_control_text(cmd_buf);
            return;
        }
    }
    for (uint16_t i = 0; i + sizeof(pid_plan_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], pid_plan_cmd, sizeof(pid_plan_cmd) - 1) == 0) {
            char cmd_buf[96];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_pid_control_text(cmd_buf);
            return;
        }
    }
    for (uint16_t i = 0; i + sizeof(pid_arm_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], pid_arm_cmd, sizeof(pid_arm_cmd) - 1) == 0) {
            char cmd_buf[32];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            handle_pid_control_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_zero_all_arm_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_zero_all_arm_cmd,
                   sizeof(motor_zero_all_arm_cmd) - 1) == 0) {
            if (Leg_Control_ArmAllZero())
                Communication_SendString("MOTOR_ZERO_ALL_ARM ok\r\n");
            else
                Communication_SendString("MOTOR_ZERO_ALL_ARM rejected\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_zero_all_run_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_zero_all_run_cmd,
                   sizeof(motor_zero_all_run_cmd) - 1) == 0) {
            if (Leg_Control_StartAllZero())
                Communication_SendString("MOTOR_ZERO_ALL_RUN ok\r\n");
            else
                Communication_SendString("MOTOR_ZERO_ALL_RUN rejected\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_stop_all_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_stop_all_cmd, sizeof(motor_stop_all_cmd) - 1) == 0) {
            Leg_Control_StopAllDebugTargets(0);
            Communication_SendString("MOTOR_STOP_ALL ok\r\n");
            Communication_SendMotorControlStatus();
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

    /* Keep the already deployed MOTOR_SET bridge compatible: it is the
       zero-offset arm command used by the PC tool before PID_PLAN is used. */
    for (uint16_t i = 0; i + sizeof(motor_set_mrad_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_set_mrad_cmd, sizeof(motor_set_mrad_cmd) - 1) == 0) {
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            for (uint16_t j = 0; j < copy_len; ++j) {
                if (cmd_buf[j] == '\r' || cmd_buf[j] == '\n') { cmd_buf[j] = '\0'; break; }
            }
            handle_motor_set_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(motor_set_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], motor_set_cmd, sizeof(motor_set_cmd) - 1) == 0) {
            char cmd_buf[48];
            uint16_t copy_len = len - i;
            if (copy_len >= sizeof(cmd_buf)) copy_len = sizeof(cmd_buf) - 1U;
            memcpy(cmd_buf, &data[i], copy_len);
            cmd_buf[copy_len] = '\0';
            for (uint16_t j = 0; j < copy_len; ++j) {
                if (cmd_buf[j] == '\r' || cmd_buf[j] == '\n') { cmd_buf[j] = '\0'; break; }
            }
            handle_motor_set_text(cmd_buf);
            return;
        }
    }

    /* Ignore unrelated UART payloads.  Motion/gait entries are deliberately
       unreachable in this PID stage, but normal link traffic must not become
       a repetitive text log. */
    return;

    for (uint16_t i = 0; i + sizeof(leg_all_micro_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_all_micro_cmd, sizeof(leg_all_micro_cmd) - 1) == 0) {
            if (!Leg_Gait_StartAllMicroTest()) {
                Communication_SendString("LEG_ALL_MICRO_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_prep_pose_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_prep_pose_cmd, sizeof(leg_prep_pose_cmd) - 1) == 0) {
            if (!Leg_Gait_StartPrepPoseTest()) {
                Communication_SendString("LEG_PREP_POSE_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_sine_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_sine_cmd, sizeof(leg_sine_cmd) - 1) == 0) {
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

            handle_leg_sine_text(cmd_buf);
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_trot_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_trot_cmd, sizeof(leg_trot_cmd) - 1) == 0) {
            if (!Leg_Gait_StartTrotTest()) {
                Communication_SendString("LEG_TROT_RX rejected\r\n");
            }
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_stand_step_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_stand_step_cmd, sizeof(leg_stand_step_cmd) - 1) == 0) {
            Communication_SendString("LEG_STAND_STEP_RX rejected (not implemented)\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_touch_step_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_touch_step_cmd, sizeof(leg_touch_step_cmd) - 1) == 0) {
            Communication_SendString("LEG_TOUCH_STEP_RX rejected (not implemented)\r\n");
            return;
        }
    }

    for (uint16_t i = 0; i + sizeof(leg_loaded_step_cmd) - 1 <= len; i++) {
        if (memcmp(&data[i], leg_loaded_step_cmd, sizeof(leg_loaded_step_cmd) - 1) == 0) {
            Communication_SendString("LEG_LOADED_STEP_RX rejected (not implemented)\r\n");
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

    (void)arm_esp32_rx();
}

static void consume_text_command_stream(const uint8_t *data, uint16_t len)
{
    if (data == NULL) return;

    for (uint16_t i = 0U; i < len; ++i) {
        uint8_t byte = data[i];
        if (byte == '\r' || byte == '\n') {
            if (text_command_len > 0U) {
                text_command_buf[text_command_len] = '\0';
                handle_motor_debug_command((const uint8_t *)text_command_buf,
                                           text_command_len);
                text_command_len = 0U;
            }
            continue;
        }

        if (byte < 0x20U || byte > 0x7EU) {
            /* A binary RC frame cannot be part of a text command. */
            text_command_len = 0U;
            continue;
        }

        if (text_command_len == 0U && byte != 'M' && byte != 'P' && byte != 'L')
            continue;

        if (text_command_len >= TEXT_COMMAND_BUF_SIZE - 1U) {
            text_command_len = 0U;
            continue;
        }
        text_command_buf[text_command_len++] = (char)byte;
    }
}

void Communication_RxCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* USART6 承载来自 ESP32 的遥控帧和文本调试指令。 */
    if (huart->Instance != USART6) return;
    if (Size == 0U) {
        restart_esp32_rx();
        return;
    }

    /* ISR work is deliberately bounded: copy this completed chunk into a
       small foreground queue and immediately re-arm USART6.  Parsing, PID
       state changes, snprintf, and UART replies run later in Communication_Task. */
    comm_ctx.rx_count++;
    comm_ctx.last_rx_size = Size;
    uint16_t snap_len = Size < RX_SNAPSHOT_SIZE ? Size : RX_SNAPSHOT_SIZE;
    memcpy((void *)comm_ctx.rx_snapshot, comm_ctx.rx_buf, snap_len);

    uint8_t write = rx_chunk_write;
    uint8_t next = (uint8_t)((write + 1U) % RX_CHUNK_QUEUE_DEPTH);
    if (next != rx_chunk_read) {
        rx_chunk_queue[write].len = Size;
        memcpy(rx_chunk_queue[write].data, comm_ctx.rx_buf, Size);
        __DMB();
        rx_chunk_write = next;
    } else {
        ++rx_chunk_overflow_count;
    }
    restart_esp32_rx();
}

void Communication_HandleUartError(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART6) return;
    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    SET_BIT(huart->Instance->RQR, UART_RXDATA_FLUSH_REQUEST);
    text_command_len = 0U;
    comm_ctx.rx_rearm_pending = 1U;
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

uint32_t Communication_GetLinkAgeMs(void)
{
    return HAL_GetTick() - last_valid_packet_tick;
}

void Communication_ResetWatchdog(void)
{
    last_valid_packet_tick = HAL_GetTick();
}

void Communication_Task(void)
{
    while (rx_chunk_read != rx_chunk_write) {
        uint8_t read = rx_chunk_read;
        const uint8_t *data = rx_chunk_queue[read].data;
        uint16_t size = rx_chunk_queue[read].len;
        handle_hello(data, size);
        consume_text_command_stream(data, size);
        int parsed_len = parse_frame(data, size);
        send_esp32_rx_echo(size, parsed_len > 0);
        __DMB();
        rx_chunk_read = (uint8_t)((read + 1U) % RX_CHUNK_QUEUE_DEPTH);
    }

    if (comm_ctx.rx_rearm_pending && comm_ctx.uart &&
        comm_ctx.uart->RxState == HAL_UART_STATE_READY) {
        restart_esp32_rx();
    }

    if (!comm_ctx.handshake_done && comm_ctx.uart &&
        (HAL_GetTick() - comm_ctx.last_hello_tick) >= 500) {
        comm_ctx.last_hello_tick = HAL_GetTick();
        Communication_SendBytes((const uint8_t *)stm32_ack, sizeof(stm32_ack) - 1);
    }

    if (comm_ctx.handshake_done && !Communication_IsLinkAlive()) {
        Communication_SetSafeRCData();
    }

    if (motor_control_status_pending && !comm_ctx.tx_busy) {
        motor_control_status_pending = 0U;
        Communication_SendMotorControlStatus();
    }
    if (leg_trajectory_status_pending && !comm_ctx.tx_busy) {
        leg_trajectory_status_pending = 0U;
        Communication_SendLegTrajectoryStatus();
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

int Communication_TrySendString(const char *str)
{
    if (!str || !comm_ctx.uart || comm_ctx.tx_busy)
        return 0;
    size_t len = strlen(str);
    if (len == 0U || len > TX_IT_BUF_SIZE)
        return 0;
    memcpy(comm_ctx.tx_buf, str, len);
    if (HAL_UART_Transmit_IT(comm_ctx.uart, comm_ctx.tx_buf,
                             (uint16_t)len) != HAL_OK)
        return 0;
    comm_ctx.tx_busy = 1U;
    return 1;
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
    static uint8_t detail_motor = 0U;
    float angles[8];
    uint8_t valid[8];
    Motor_StateSnapshotTypeDef state;
    Leg_Control_GetAngles(angles, valid);
    if (!Leg_Control_GetMotorStateSnapshot(detail_motor, &state))
        return;

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
        "%u,%u,%u,%u,%u,%u,%u,%u S%u=",
        valid[0], valid[1], valid[2], valid[3],
        valid[4], valid[5], valid[6], valid[7], detail_motor);
        if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
            len = -1;
        else
            len += written;
    }

    /* Compatibility prefix contains eight multi-turn rotor positions.  The
       optional S suffix is self-describing diagnostics for one motor. */
    const float detail[4] = {
        state.rotor_position,
        state.zero_rotor_position,
        state.joint_position,
        state.direction,
    };
    for (uint8_t i = 0U; i < 4U && len > 0; ++i) {
        len = append_fixed4(buf, sizeof(buf), len, detail[i]);
        if (len > 0) {
            int written = snprintf(&buf[len], sizeof(buf) - (size_t)len,
                                   "%c", (i == 3U) ? '\n' : ',');
            if (written < 0 || written >= (int)(sizeof(buf) - (size_t)len))
                len = -1;
            else
                len += written;
        }
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
        detail_motor = (uint8_t)((detail_motor + 1U) & 0x07U);
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

static int append_motor_health(char *buf, size_t size, int len)
{
    int8_t temperature[8] = {0};
    uint8_t motor_error[8] = {0};
    uint8_t valid[8] = {0};

    for (uint8_t i = 0U; i < 8U; ++i) {
        Motor_StateSnapshotTypeDef state;
        if (Leg_Control_GetMotorStateSnapshot(i, &state)) {
            temperature[i] = state.temperature_c;
            motor_error[i] = state.motor_error;
            valid[i] = (state.online && state.angle_valid) ? 1U : 0U;
        }
    }

    if (len < 0 || (size_t)len >= size)
        return -1;

    int written = snprintf(&buf[len], size - (size_t)len,
        "MOTOR_HEALTH %d,%d,%d,%d,%d,%d,%d,%d "
        "%u,%u,%u,%u,%u,%u,%u,%u %u,%u,%u,%u,%u,%u,%u,%u\n",
        (int)temperature[0], (int)temperature[1],
        (int)temperature[2], (int)temperature[3],
        (int)temperature[4], (int)temperature[5],
        (int)temperature[6], (int)temperature[7],
        motor_error[0], motor_error[1], motor_error[2], motor_error[3],
        motor_error[4], motor_error[5], motor_error[6], motor_error[7],
        valid[0], valid[1], valid[2], valid[3],
        valid[4], valid[5], valid[6], valid[7]);
    if (written < 0 || written >= (int)(size - (size_t)len))
        return -1;
    return len + written;
}

static int append_motor_group(char *buf, size_t size, int len)
{
    Motor_GroupSnapshot group;
    Leg_Control_GetGroupSnapshot(&group);
    if (len < 0 || (size_t)len >= size) return -1;

    int written = snprintf(&buf[len], size - (size_t)len,
        "MOTOR_GROUP mode=%u reason=%u ready=%u at_zero=%u offset=",
        (unsigned int)group.mode, (unsigned int)group.reason,
        (unsigned int)group.ready, (unsigned int)group.all_at_zero);
    if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
    len += written;

    len = append_fixed4(buf, size, len, group.target_offset);
    if (len <= 0) return -1;
    written = snprintf(&buf[len], size - (size_t)len, " delta=");
    if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
    len += written;

    for (uint8_t i = 0U; i < 8U && len > 0; ++i) {
        len = append_fixed4(buf, size, len,
                            group.target_position[i] - group.arm_position[i]);
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len,
                               "%c", i == 7U ? ' ' : ',');
            if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
            len += written;
        }
    }
    if (len > 0) {
        written = snprintf(&buf[len], size - (size_t)len, "error=");
        if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
        len += written;
    }
    for (uint8_t i = 0U; i < 8U && len > 0; ++i) {
        len = append_fixed4(buf, size, len, group.position_error[i]);
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len,
                               "%c", i == 7U ? ' ' : ',');
            if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
            len += written;
        }
    }
    if (len > 0) {
        written = snprintf(&buf[len], size - (size_t)len, "profile=%u\n",
                           (unsigned int)group.profile);
        if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
        len += written;
    }
    return len;
}

static float clamp_diagnostic_value(float value, float maximum)
{
    if (!isfinite(value) || value < 0.0f) return 0.0f;
    return value > maximum ? maximum : value;
}

static int append_motor_group_diagnostics(char *buf, size_t size, int len)
{
    Motor_GroupDiagnostics diagnostics;
    Leg_Control_GetGroupDiagnostics(&diagnostics, 1U);
    if (len < 0 || (size_t)len >= size) return -1;

    int written = snprintf(&buf[len], size - (size_t)len,
                           "MOTOR_GROUP_DIAG p2p=");
    if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
    len += written;
    for (uint8_t i = 0U; i < 8U && len > 0; ++i) {
        len = append_fixed4(buf, size, len,
                            clamp_diagnostic_value(
                                diagnostics.position_peak_to_peak[i], 9.9999f));
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len,
                               "%c", i == 7U ? ' ' : ',');
            if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
            len += written;
        }
    }

    if (len > 0) {
        written = snprintf(&buf[len], size - (size_t)len, "vmax=");
        if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
        len += written;
    }
    for (uint8_t i = 0U; i < 8U && len > 0; ++i) {
        len = append_fixed4(buf, size, len,
                            clamp_diagnostic_value(
                                diagnostics.max_abs_velocity[i], 99.9999f));
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len,
                               "%c", i == 7U ? ' ' : ',');
            if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
            len += written;
        }
    }

    if (len > 0) {
        written = snprintf(&buf[len], size - (size_t)len, "tmax=");
        if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
        len += written;
    }
    for (uint8_t i = 0U; i < 8U && len > 0; ++i) {
        len = append_fixed4(buf, size, len,
                            clamp_diagnostic_value(
                                diagnostics.max_abs_torque[i], 1.5000f));
        if (len > 0) {
            written = snprintf(&buf[len], size - (size_t)len,
                               "%c", i == 7U ? '\n' : ',');
            if (written < 0 || written >= (int)(size - (size_t)len)) return -1;
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
        len = append_motor_group(buf, sizeof(buf), len);
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        len = append_motor_group_diagnostics(buf, sizeof(buf), len);
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        len = append_motor_health(buf, sizeof(buf), len);
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        len = append_motor_zero(buf, sizeof(buf), len);
    }

    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}

static int parse_pid_arm_command(const char *cmd, int *motor)
{
    const char *p = cmd;
    if (memcmp(p, pid_arm_cmd, sizeof(pid_arm_cmd) - 1) != 0) return 0;
    p += sizeof(pid_arm_cmd) - 1;
    if (*p != ' ' && *p != '\t') return 0;
    if (!parse_int_value(&p, motor)) return 0;
    p = skip_spaces(p);
    return *p == '\0';
}

static int parse_pid_hold_dryrun_command(const char *cmd, int *motor)
{
    const char *p = cmd;
    if (memcmp(p, pid_hold_dryrun_cmd,
               sizeof(pid_hold_dryrun_cmd) - 1) != 0) return 0;
    p += sizeof(pid_hold_dryrun_cmd) - 1;
    if (*p != ' ' && *p != '\t') return 0;
    if (!parse_int_value(&p, motor)) return 0;
    p = skip_spaces(p);
    return *p == '\0';
}

static int parse_pid_hold_active_command(const char *cmd, int *motor)
{
    const char *p = cmd;
    if (memcmp(p, pid_hold_active_cmd,
               sizeof(pid_hold_active_cmd) - 1) != 0) return 0;
    p += sizeof(pid_hold_active_cmd) - 1;
    if (*p != ' ' && *p != '\t') return 0;
    if (!parse_int_value(&p, motor)) return 0;
    p = skip_spaces(p);
    return *p == '\0';
}

static int parse_pid_traj_dryrun_command(const char *cmd, int *motor)
{
    const char *p = cmd;
    if (memcmp(p, pid_traj_dryrun_cmd,
               sizeof(pid_traj_dryrun_cmd) - 1) != 0) return 0;
    p += sizeof(pid_traj_dryrun_cmd) - 1;
    if (*p != ' ' && *p != '\t') return 0;
    if (!parse_int_value(&p, motor)) return 0;
    p = skip_spaces(p);
    return *p == '\0';
}

static int parse_pid_traj_active_command(const char *cmd, int *motor)
{
    const char *p = cmd;
    if (memcmp(p, pid_traj_active_cmd,
               sizeof(pid_traj_active_cmd) - 1) != 0) return 0;
    p += sizeof(pid_traj_active_cmd) - 1;
    if (*p != ' ' && *p != '\t') return 0;
    if (!parse_int_value(&p, motor)) return 0;
    p = skip_spaces(p);
    return *p == '\0';
}

static void send_static_hold_plan(void)
{
    Motor_SoftwareControlSnapshot s;
    Motor_SoftwareControl_GetSnapshot(&s);
    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "SW_HOLD_PLAN {\"dry\":%u,\"i\":%d,\"rel\":0,"
        "\"pkp\":%ld,\"pki\":%ld,\"pkd\":%ld,"
        "\"skp\":%ld,\"ski\":%ld,\"skd\":%ld,"
        "\"tmax\":%ld,\"vt\":%ld,\"vm\":%ld,"
        "\"xm\":%ld,\"dur\":%lu,\"dkp\":0,\"dkw\":0,"
        "\"vf\":%ld,\"ig\":%ld,\"cal\":0,\"sa\":383,\"od\":0,\"guard\":%u}\n",
        (unsigned int)s.dry_run, (int)s.motor_index,
        (long)(s.position_loop_kp * 1000000.0f),
        (long)(s.position_loop_ki * 1000000.0f),
        (long)(s.position_loop_kd * 1000000.0f),
        (long)(s.speed_loop_kp * 1000000.0f),
        (long)(s.speed_loop_ki * 1000000.0f),
        (long)(s.speed_loop_kd * 1000000.0f),
        (long)(s.torque_limit * 1000000.0f),
        (long)(s.target_velocity_limit * 1000000.0f),
        (long)(s.actual_velocity_limit * 1000000.0f),
        (long)(s.position_excursion_limit * 1000000.0f),
        (unsigned long)s.duration_ms,
        (long)(s.velocity_filter_hz * 1000000.0f),
        (long)(s.integral_position_gate * 1000000.0f),
        (unsigned int)Motor_Transport_IsZeroOutputOnly());
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

static void send_single_motor_trajectory_plan(void)
{
    Motor_SoftwareControlSnapshot s;
    Motor_SoftwareControl_GetSnapshot(&s);
    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(
        buf, sizeof(buf),
        "SW_TRAJ_PLAN {\"dry\":%u,\"i\":%d,\"amp\":%ld,"
        "\"period\":%lu,\"cycles\":%u,\"settle\":%u,"
        "\"pkp\":%ld,\"pkd\":%ld,\"skp\":%ld,\"ski\":%ld,"
        "\"skd\":%ld,\"tmax\":%ld,\"vt\":%ld,\"vm\":%ld,"
        "\"xm\":%ld,\"dur\":%lu,\"guard\":%u}\n",
        (unsigned int)s.dry_run, (int)s.motor_index,
        (long)(s.trajectory_amplitude * 1000000.0f),
        (unsigned long)s.trajectory_period_ms,
        (unsigned int)s.trajectory_cycles,
        (unsigned int)MOTOR_SOFTWARE_TRAJECTORY_SETTLE_MS,
        (long)(s.position_loop_kp * 1000000.0f),
        (long)(s.position_loop_kd * 1000000.0f),
        (long)(s.speed_loop_kp * 1000000.0f),
        (long)(s.speed_loop_ki * 1000000.0f),
        (long)(s.speed_loop_kd * 1000000.0f),
        (long)(s.torque_limit * 1000000.0f),
        (long)(s.target_velocity_limit * 1000000.0f),
        (long)(s.actual_velocity_limit * 1000000.0f),
        (long)(s.position_excursion_limit * 1000000.0f),
        (unsigned long)s.duration_ms,
        (unsigned int)Motor_Transport_IsZeroOutputOnly());
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

static int parse_pid_plan_command(const char *cmd, int *motor, int *offset_mrad,
                                  int *kp_milli, int *kw_milli, int *duration_ms,
                                  const char **reason)
{
    const char *p = cmd;
    *reason = "prefix";
    if (memcmp(p, pid_plan_cmd, sizeof(pid_plan_cmd) - 1) != 0) return 0;
    p += sizeof(pid_plan_cmd) - 1;
    *reason = "separator";
    if (*p != ' ' && *p != '\t') return 0;
    *reason = "motor";
    if (!parse_int_value(&p, motor)) return 0;
    *reason = "offset_mrad";
    if (!parse_int_value(&p, offset_mrad)) return 0;
    *reason = "kp_milli";
    if (!parse_int_value(&p, kp_milli)) return 0;
    *reason = "kw_milli";
    if (!parse_int_value(&p, kw_milli)) return 0;
    *reason = "duration_ms";
    if (!parse_int_value(&p, duration_ms)) return 0;
    p = skip_spaces(p);
    *reason = "trailing";
    if (*p != '\0') return 0;
    *reason = "ok";
    return 1;
}

static void handle_pid_control_text(const char *cmd_buf)
{
    int motor = -1, offset_mrad = 0, kp_milli = 0, kw_milli = 0, duration_ms = 0;
    int accepted = 0;
    const char *parse_reason = "arm_or_plan";
    Motor_ControlSnapshotTypeDef control_before;
    Motor_StateSnapshotTypeDef motor_before;
    memset(&control_before, 0, sizeof(control_before));
    memset(&motor_before, 0, sizeof(motor_before));
    Leg_Control_GetControlSnapshot(&control_before);
    if (parse_pid_arm_command(cmd_buf, &motor)) {
        if (motor >= 0 && motor < 8)
            accepted = Leg_Control_ArmSingleMotor((uint8_t)motor);
    } else if (parse_pid_hold_dryrun_command(cmd_buf, &motor)) {
        if (motor >= 0 && motor < 8)
            accepted = Leg_Control_StartStaticHoldDryRun((uint8_t)motor);
    } else if (parse_pid_hold_active_command(cmd_buf, &motor)) {
        if (motor >= 0 && motor < 8)
            accepted = Leg_Control_StartStaticHoldActive((uint8_t)motor);
    } else if (parse_pid_traj_dryrun_command(cmd_buf, &motor)) {
        if (motor >= 0 && motor < 8)
            accepted = Leg_Control_StartSingleMotorTrajectoryDryRun(
                (uint8_t)motor);
    } else if (parse_pid_traj_active_command(cmd_buf, &motor)) {
        if (motor >= 0 && motor < 8)
            accepted = Leg_Control_StartSingleMotorTrajectoryActive(
                (uint8_t)motor);
    } else if (parse_pid_plan_command(cmd_buf, &motor, &offset_mrad, &kp_milli,
                                      &kw_milli, &duration_ms, &parse_reason)) {
        if (motor >= 0 && motor < 8)
            accepted = Leg_Control_PlanSingleMotor((uint8_t)motor,
                        (float)offset_mrad / 1000.0f, (float)kp_milli / 1000.0f,
                        (float)kw_milli / 1000.0f, (uint32_t)duration_ms);
    } else {
        char buf[144];
        int len = snprintf(buf, sizeof(buf), "PID_RX rejected reason=%s raw=%s\r\n",
                           parse_reason, cmd_buf);
        if (len > 0 && len < (int)sizeof(buf))
            Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
        return;
    }
    if (!accepted) {
        int has_motor = (motor >= 0 && motor < 8) &&
                        Leg_Control_GetMotorStateSnapshot((uint8_t)motor, &motor_before);
        char buf[192];
        int len = snprintf(buf, sizeof(buf),
                           "PID_PLAN rejected gate=%s prior_mode=%u prior_reason=%u "
                           "armed=%d req=%d off_mrad=%d kp_milli=%d kw_milli=%d "
                           "dur=%d online=%u valid=%u\r\n",
                           Leg_Control_GetLastPlanRejectReason(),
                           (unsigned int)control_before.mode,
                           (unsigned int)control_before.reason,
                           (int)control_before.armed_motor_index, motor, offset_mrad,
                           kp_milli, kw_milli, duration_ms,
                           has_motor ? (unsigned int)motor_before.online : 0U,
                           has_motor ? (unsigned int)motor_before.angle_valid : 0U);
        if (len > 0 && len < (int)sizeof(buf))
            Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
    if (accepted && (parse_pid_hold_dryrun_command(cmd_buf, &motor) ||
                     parse_pid_hold_active_command(cmd_buf, &motor)))
        send_static_hold_plan();
    if (accepted && (parse_pid_traj_dryrun_command(cmd_buf, &motor) ||
                     parse_pid_traj_active_command(cmd_buf, &motor)))
        send_single_motor_trajectory_plan();
    Communication_SendMotorControlStatus();
}

void Communication_SendMotorControlStatus(void)
{
    if (!comm_ctx.uart || comm_ctx.tx_busy) {
        motor_control_status_pending = 1U;
        return;
    }

    Motor_ControlSnapshotTypeDef control;
    Leg_Control_GetControlSnapshot(&control);

    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "MOTOR_CONTROL mode=%u reason=%u guard=%u armed=%d target=",
        (unsigned int)control.mode, (unsigned int)control.reason,
        (unsigned int)control.zero_output_guard, (int)control.armed_motor_index);
    len = append_fixed4(buf, sizeof(buf), len, control.target_rotor_position);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " actual=");
    len = append_fixed4(buf, sizeof(buf), len, control.actual_rotor_position);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " target_joint=");
    len = append_fixed4(buf, sizeof(buf), len, control.target_joint_position);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " actual_joint=");
    len = append_fixed4(buf, sizeof(buf), len, control.actual_joint_position);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " error=");
    len = append_fixed4(buf, sizeof(buf), len, control.position_error);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " kp=");
    len = append_fixed4(buf, sizeof(buf), len, control.kp);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len, " kw=");
    len = append_fixed4(buf, sizeof(buf), len, control.kw);
    if (len > 0) len += snprintf(&buf[len], sizeof(buf) - (size_t)len,
                                 " duration=%lu\n", (unsigned long)control.duration_ms);
    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}

void Communication_SendSoftwarePidTelemetry(void)
{
    static uint8_t phase = 0U;
    if (!comm_ctx.uart || comm_ctx.tx_busy) return;
    Motor_SoftwareControlSnapshot s;
    Motor_SoftwareControl_GetSnapshot(&s);
    if (s.mode != Motor_SoftwareControl_DryRun &&
        s.mode != Motor_SoftwareControl_ActiveTorque &&
        s.mode != Motor_SoftwareControl_CascadeDryRun &&
        s.mode != Motor_SoftwareControl_CascadeActiveTorque &&
        s.mode != Motor_SoftwareControl_StaticHoldDryRun &&
        s.mode != Motor_SoftwareControl_StaticHoldActiveTorque &&
        s.mode != Motor_SoftwareControl_TrajectoryDryRun &&
        s.mode != Motor_SoftwareControl_TrajectoryActiveTorque &&
        s.mode != Motor_SoftwareControl_Stopped) return;

    /* Alternate two bounded records. Each remains comfortably below the
       256-byte STM32 line buffer even for expanded multi-radian targets. */
    char buf[TX_IT_BUF_SIZE];
    int len;
    if (s.test == Motor_SoftwareControl_TestCycloidTrajectory && phase == 0U) {
        len = snprintf(buf, sizeof(buf),
            "SW_TRAJ_STATE {\"m\":%u,\"i\":%d,\"ph\":%ld,\"cy\":%u,"
            "\"done\":%u,\"rt\":%ld,\"p\":%ld,\"e\":%ld,"
            "\"v\":%ld,\"fv\":%ld,\"rv\":%ld,\"dt\":%lu,"
            "\"el\":%lu,\"stop\":%u,\"sl\":%u}\n",
            (unsigned int)s.mode, (int)s.motor_index,
            (long)(s.trajectory_phase * 1000000.0f),
            (unsigned int)s.trajectory_cycle_index,
            (unsigned int)s.trajectory_complete,
            (long)(s.ramped_target * 1000000.0f),
            (long)(s.actual_position * 1000000.0f),
            (long)(s.position_error * 1000000.0f),
            (long)(s.raw_velocity * 1000000.0f),
            (long)(s.filtered_velocity * 1000000.0f),
            (long)(s.ramp_velocity * 1000000.0f),
            (unsigned long)(s.dt_s * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.stop_reason,
            (unsigned int)s.safety_limit);
    } else if (s.test == Motor_SoftwareControl_TestCycloidTrajectory) {
        int32_t torque_q8 = (int32_t)(s.limited_torque * 256.0f);
        len = snprintf(buf, sizeof(buf),
            "SW_TRAJ_CTRL {\"m\":%u,\"i\":%d,\"P\":%ld,\"I\":%ld,"
            "\"D\":%ld,\"T\":%ld,\"qT\":%ld,\"lim\":%u,"
            "\"st\":%ld,\"se\":%ld,\"el\":%lu,\"stop\":%u,"
            "\"sl\":%u}\n",
            (unsigned int)s.mode, (int)s.motor_index,
            (long)(s.p_term * 1000000.0f),
            (long)(s.i_term * 1000000.0f),
            (long)(s.d_term * 1000000.0f),
            (long)(s.limited_torque * 1000000.0f),
            (long)((float)torque_q8 * (1000000.0f / 256.0f)),
            (unsigned int)s.torque_limited,
            (long)(s.speed_target * 1000000.0f),
            (long)(s.speed_error * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.stop_reason,
            (unsigned int)s.safety_limit);
    } else if (s.test == Motor_SoftwareControl_TestStaticHold && phase == 0U) {
        float deviation = s.actual_position - s.arm_position;
        float excursion = deviation >= 0.0f ? deviation : -deviation;
        len = snprintf(buf, sizeof(buf),
            "SW_HOLD_STATE {\"m\":%u,\"i\":%d,\"a\":%ld,\"p\":%ld,"
            "\"dev\":%ld,\"x\":%ld,\"sim\":%ld,\"ph\":%u,\"e\":%ld,"
            "\"v\":%ld,\"fv\":%ld,\"vb\":%ld,\"cv\":%ld,\"pv\":%ld,"
            "\"bv\":%u,\"oc\":%u,\"ft\":%ld,\"dt\":%lu,"
            "\"el\":%lu,\"stop\":%u,\"sl\":%u}\n",
            (unsigned int)s.mode, (int)s.motor_index,
            (long)(s.arm_position * 1000000.0f),
            (long)(s.actual_position * 1000000.0f),
            (long)(deviation * 1000000.0f),
            (long)(excursion * 1000000.0f),
            (long)(s.simulated_position_offset * 1000000.0f),
            (unsigned int)s.simulation_phase,
            (long)(s.position_error * 1000000.0f),
            (long)(s.raw_velocity * 1000000.0f),
            (long)(s.filtered_velocity * 1000000.0f),
            (long)(s.velocity_bias * 1000000.0f),
            (long)(s.corrected_velocity * 1000000.0f),
            (long)(s.position_velocity * 1000000.0f),
            (unsigned int)s.velocity_bias_valid,
            (unsigned int)s.raw_overspeed_count,
            (long)(s.feedback_torque * 1000000.0f),
            (unsigned long)(s.dt_s * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.stop_reason,
            (unsigned int)s.safety_limit);
    } else if (s.test == Motor_SoftwareControl_TestStaticHold) {
        int32_t torque_q8 = (int32_t)(s.limited_torque * 256.0f);
        len = snprintf(buf, sizeof(buf),
            "SW_HOLD_CTRL {\"m\":%u,\"i\":%d,\"st\":%ld,\"se\":%ld,"
            "\"P\":%ld,\"I\":%ld,\"D\":%ld,\"T\":%ld,\"qT\":%ld,\"lim\":%u,\"ie\":%u,"
            "\"el\":%lu,\"stop\":%u,\"sl\":%u}\n",
            (unsigned int)s.mode, (int)s.motor_index,
            (long)(s.speed_target * 1000000.0f),
            (long)(s.speed_error * 1000000.0f),
            (long)(s.p_term * 1000000.0f), (long)(s.i_term * 1000000.0f),
            (long)(s.d_term * 1000000.0f),
            (long)(s.limited_torque * 1000000.0f),
            (long)((float)torque_q8 * (1000000.0f / 256.0f)),
            (unsigned int)s.torque_limited,
            (unsigned int)s.integral_enabled,
            (unsigned long)s.elapsed_ms, (unsigned int)s.stop_reason,
            (unsigned int)s.safety_limit);
    } else if (phase == 0U) {
        len = snprintf(buf, sizeof(buf),
            "SW_PID_STATE {\"m\":%u,\"i\":%d,\"rt\":%ld,\"sp\":%ld,"
            "\"p\":%ld,\"e\":%ld,\"v\":%ld,\"fv\":%ld,\"rv\":%ld,\"dt\":%lu,"
            "\"el\":%lu,\"stop\":%u}\n",
            (unsigned int)s.mode, (int)s.motor_index,
            (long)(s.raw_target * 1000000.0f),
            (long)(s.ramped_target * 1000000.0f),
            (long)(s.actual_position * 1000000.0f),
            (long)(s.position_error * 1000000.0f),
            (long)(s.raw_velocity * 1000000.0f),
            (long)(s.filtered_velocity * 1000000.0f),
            (long)(s.ramp_velocity * 1000000.0f),
            (unsigned long)(s.dt_s * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.stop_reason);
    } else {
        int32_t torque_q8 = (int32_t)(s.limited_torque * 256.0f);
        len = snprintf(buf, sizeof(buf),
            "SW_PID_CTRL {\"m\":%u,\"i\":%d,\"P\":%ld,\"I\":%ld,"
            "\"D\":%ld,\"T\":%ld,\"qT\":%ld,\"lim\":%u,\"st\":%ld,\"se\":%ld,"
            "\"el\":%lu,\"stop\":%u}\n",
            (unsigned int)s.mode, (int)s.motor_index,
            (long)(s.p_term * 1000000.0f), (long)(s.i_term * 1000000.0f),
            (long)(s.d_term * 1000000.0f),
            (long)(s.limited_torque * 1000000.0f),
            (long)((float)torque_q8 * (1000000.0f / 256.0f)),
            (unsigned int)s.torque_limited,
            (long)(s.speed_target * 1000000.0f),
            (long)(s.speed_error * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.stop_reason);
    }
    phase ^= 1U;
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

void Communication_SendLegTrajectoryStatus(void)
{
    if (!comm_ctx.uart || comm_ctx.tx_busy) {
        leg_trajectory_status_pending = 1U;
        return;
    }
    Motor_LegTrajectorySnapshot s;
    Leg_Control_GetRfLegTrajectorySnapshot(&s);
    char buf[192];
    int len = snprintf(
        buf, sizeof(buf),
        "LEG_TRAJ_STATUS mode=%u reason=%u dry_ok=%u guard=%u level=%u "
        "active=%u elapsed=%lu idx=%d detail=%ld seq=%lu hold=%u\n",
        (unsigned int)s.mode, (unsigned int)s.reason,
        (unsigned int)s.dry_run_passed,
        (unsigned int)Motor_Transport_IsZeroOutputOnly(),
        (unsigned int)s.profile.level,
        (unsigned int)MOTOR_LEG_TRAJECTORY_ACTIVE_ENABLED,
        (unsigned long)s.elapsed_ms, (int)s.stop_motor_index,
        (long)(s.stop_detail * 1000000.0f),
        (unsigned long)s.stop_sequence,
        (unsigned int)s.hold_current_position);
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

int Communication_TrySendLegTrajectoryHoldResult(void)
{
    Motor_LegTrajectorySnapshot s;
    Leg_Control_GetRfLegTrajectorySnapshot(&s);
    if (s.mode != Motor_LegTrajectory_Hold)
        return 0;

    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(
        buf, sizeof(buf),
        "LEG_TRAJ_HOLD_RESULT {\"level\":%u,\"m\":%u,\"reason\":%u,\"hold\":%u,"
        "\"out\":1,\"el\":%lu,\"samples\":[%lu,%lu],"
        "\"d\":[%ld,%ld],\"tv\":[%ld,%ld],\"av\":[%ld,%ld],"
        "\"err\":[%ld,%ld],\"tq\":[%ld,%ld],\"sat\":[%lu,%lu],"
        "\"dt_min\":[%ld,%ld],\"dt_max\":[%ld,%ld],"
        "\"target\":[%ld,%ld],\"actual\":[%ld,%ld]}\n",
        (unsigned int)s.profile.level,
        (unsigned int)s.mode, (unsigned int)s.reason,
        (unsigned int)s.hold_current_position,
        (unsigned long)s.elapsed_ms,
        (unsigned long)s.feedback_count[0],
        (unsigned long)s.feedback_count[1],
        (long)(s.peak_target_delta[0] * 1000000.0f),
        (long)(s.peak_target_delta[1] * 1000000.0f),
        (long)(s.peak_abs_target_velocity[0] * 1000000.0f),
        (long)(s.peak_abs_target_velocity[1] * 1000000.0f),
        (long)(s.peak_abs_actual_velocity[0] * 1000000.0f),
        (long)(s.peak_abs_actual_velocity[1] * 1000000.0f),
        (long)(s.peak_abs_position_error[0] * 1000000.0f),
        (long)(s.peak_abs_position_error[1] * 1000000.0f),
        (long)(s.peak_abs_torque[0] * 1000000.0f),
        (long)(s.peak_abs_torque[1] * 1000000.0f),
        (unsigned long)s.torque_limit_count[0],
        (unsigned long)s.torque_limit_count[1],
        (long)(s.min_dt_ms[0] * 1000.0f),
        (long)(s.min_dt_ms[1] * 1000.0f),
        (long)(s.max_dt_ms[0] * 1000.0f),
        (long)(s.max_dt_ms[1] * 1000.0f),
        (long)(s.target_position[0] * 1000000.0f),
        (long)(s.target_position[1] * 1000000.0f),
        (long)(s.actual_position[0] * 1000000.0f),
        (long)(s.actual_position[1] * 1000000.0f));
    if (len <= 0 || len >= (int)sizeof(buf))
        return 0;
    return Communication_TrySendString(buf);
}

void Communication_SendLegTrajectoryTelemetry(void)
{
    static uint8_t phase = 0U;
    if (!comm_ctx.uart || comm_ctx.tx_busy) return;
    Motor_LegTrajectorySnapshot s;
    Leg_Control_GetRfLegTrajectorySnapshot(&s);
    if (s.mode != Motor_LegTrajectory_DryRun &&
        s.mode != Motor_LegTrajectory_Active &&
        s.mode != Motor_LegTrajectory_Hold &&
        s.mode != Motor_LegTrajectory_Stopped) return;

    char buf[TX_IT_BUF_SIZE];
    int len;
    if (s.mode == Motor_LegTrajectory_Stopped) {
        len = snprintf(
            buf, sizeof(buf),
            "LEG_TRAJ_RESULT {\"level\":%u,\"m\":%u,\"reason\":%u,\"dry_ok\":%u,"
            "\"out\":0,\"el\":%lu,\"idx\":%d,\"detail\":%ld,"
            "\"samples\":[%lu,%lu],\"d\":[%ld,%ld],"
            "\"tv\":[%ld,%ld],\"av\":[%ld,%ld],"
            "\"err\":[%ld,%ld],\"tq\":[%ld,%ld],"
            "\"sat\":[%lu,%lu],\"dt_min\":[%ld,%ld],"
            "\"dt_max\":[%ld,%ld]}\n",
            (unsigned int)s.profile.level,
            (unsigned int)s.mode, (unsigned int)s.reason,
            (unsigned int)s.dry_run_passed, (unsigned long)s.elapsed_ms,
            (int)s.stop_motor_index,
            (long)(s.stop_detail * 1000000.0f),
            (unsigned long)s.feedback_count[0],
            (unsigned long)s.feedback_count[1],
            (long)(s.peak_target_delta[0] * 1000000.0f),
            (long)(s.peak_target_delta[1] * 1000000.0f),
            (long)(s.peak_abs_target_velocity[0] * 1000000.0f),
            (long)(s.peak_abs_target_velocity[1] * 1000000.0f),
            (long)(s.peak_abs_actual_velocity[0] * 1000000.0f),
            (long)(s.peak_abs_actual_velocity[1] * 1000000.0f),
            (long)(s.peak_abs_position_error[0] * 1000000.0f),
            (long)(s.peak_abs_position_error[1] * 1000000.0f),
            (long)(s.peak_abs_torque[0] * 1000000.0f),
            (long)(s.peak_abs_torque[1] * 1000000.0f),
            (unsigned long)s.torque_limit_count[0],
            (unsigned long)s.torque_limit_count[1],
            (long)(s.min_dt_ms[0] * 1000.0f),
            (long)(s.min_dt_ms[1] * 1000.0f),
            (long)(s.max_dt_ms[0] * 1000.0f),
            (long)(s.max_dt_ms[1] * 1000.0f));
    } else if (phase == 0U) {
        len = snprintf(
            buf, sizeof(buf),
            "LEG_TRAJ_STATE {\"m\":%u,\"ph\":%ld,\"done\":%u,"
            "\"x\":%ld,\"y\":%ld,\"rt0\":%ld,\"p0\":%ld,"
            "\"e0\":%ld,\"v0\":%ld,\"rv0\":%ld,\"rt1\":%ld,"
            "\"p1\":%ld,\"e1\":%ld,\"v1\":%ld,\"rv1\":%ld,"
            "\"el\":%lu,\"stop\":%u}\n",
            (unsigned int)s.mode, (long)(s.phase * 1000000.0f),
            (unsigned int)s.trajectory_complete,
            (long)(s.target_foot.x * 1000.0f),
            (long)(s.target_foot.y * 1000.0f),
            (long)(s.target_position[0] * 1000000.0f),
            (long)(s.actual_position[0] * 1000000.0f),
            (long)(s.position_error[0] * 1000000.0f),
            (long)(s.raw_velocity[0] * 1000000.0f),
            (long)(s.target_velocity[0] * 1000000.0f),
            (long)(s.target_position[1] * 1000000.0f),
            (long)(s.actual_position[1] * 1000000.0f),
            (long)(s.position_error[1] * 1000000.0f),
            (long)(s.raw_velocity[1] * 1000000.0f),
            (long)(s.target_velocity[1] * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.reason);
    } else {
        int32_t tq0 = (int32_t)(s.torque[0] * 256.0f);
        int32_t tq1 = (int32_t)(s.torque[1] * 256.0f);
        len = snprintf(
            buf, sizeof(buf),
            "LEG_TRAJ_CTRL {\"m\":%u,\"P0\":%ld,\"I0\":%ld,"
            "\"D0\":%ld,\"T0\":%ld,\"qT0\":%ld,\"lim0\":%u,"
            "\"st0\":%ld,\"se0\":%ld,\"P1\":%ld,\"I1\":%ld,"
            "\"D1\":%ld,\"T1\":%ld,\"qT1\":%ld,\"lim1\":%u,"
            "\"st1\":%ld,\"se1\":%ld,\"el\":%lu,\"stop\":%u,"
            "\"idx\":%d}\n",
            (unsigned int)s.mode,
            (long)(s.p_term[0] * 1000000.0f),
            (long)(s.i_term[0] * 1000000.0f),
            (long)(s.d_term[0] * 1000000.0f),
            (long)(s.torque[0] * 1000000.0f),
            (long)((float)tq0 * (1000000.0f / 256.0f)),
            (unsigned int)s.torque_limited[0],
            (long)(s.speed_target[0] * 1000000.0f),
            (long)(s.speed_error[0] * 1000000.0f),
            (long)(s.p_term[1] * 1000000.0f),
            (long)(s.i_term[1] * 1000000.0f),
            (long)(s.d_term[1] * 1000000.0f),
            (long)(s.torque[1] * 1000000.0f),
            (long)((float)tq1 * (1000000.0f / 256.0f)),
            (unsigned int)s.torque_limited[1],
            (long)(s.speed_target[1] * 1000000.0f),
            (long)(s.speed_error[1] * 1000000.0f),
            (unsigned long)s.elapsed_ms, (unsigned int)s.reason,
            (int)s.stop_motor_index);
    }
    if (s.mode != Motor_LegTrajectory_Stopped) phase ^= 1U;
    if (len > 0 && len < (int)sizeof(buf))
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
}

void Communication_SendMotorTransportStatus(void)
{
    static uint8_t channel_index = 0;
    static const char *const channel_name[4] = {"LF", "RF", "LB", "RB"};
    Motor_TransportStats stats;

    if (!Motor_Transport_GetStats(channel_index, &stats)) {
        channel_index = 0;
        return;
    }

    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "MOTOR_TRANSPORT %s run=%u tx=%lu/%lu rx=%lu/%lu miss=%lu/%lu "
        "busy=%lu txe=%lu crc=%lu id=%lu resync=%lu uart=%lu rst=%lu "
        "pend=%u tb=%u r=%u/%u ov=%lu\n",
        channel_name[channel_index],
        stats.running,
        stats.tx_count[0], stats.tx_count[1],
        stats.rx_count[0], stats.rx_count[1],
        stats.miss_count[0], stats.miss_count[1],
        stats.busy_count,
        stats.tx_error_count,
        stats.crc_error_count,
        stats.id_error_count,
        stats.resync_count,
        stats.uart_error_count,
        stats.restart_count,
        stats.pending_motor,
        stats.tx_busy,
        stats.rx_read_index,
        stats.rx_write_index,
        stats.schedule_overrun_count);

    channel_index = (uint8_t)((channel_index + 1U) & 0x03U);
    if (len > 0 && len < (int)sizeof(buf)) {
        Communication_SendBytes((const uint8_t *)buf, (uint16_t)len);
    }
}

void Communication_SendMotorTransportSummaryBlocking(void)
{
    Motor_TransportStats stats[4];
    for (uint8_t i = 0; i < 4U; ++i) {
        if (!Motor_Transport_GetStats(i, &stats[i])) {
            return;
        }
    }

    uint32_t busy = 0U;
    uint32_t txe = 0U;
    uint32_t crc = 0U;
    uint32_t id = 0U;
    uint32_t resync = 0U;
    uint32_t uart = 0U;
    uint32_t rst = 0U;
    for (uint8_t i = 0; i < 4U; ++i) {
        busy += stats[i].busy_count;
        txe += stats[i].tx_error_count;
        crc += stats[i].crc_error_count;
        id += stats[i].id_error_count;
        resync += stats[i].resync_count;
        uart += stats[i].uart_error_count;
        rst += stats[i].restart_count;
    }

    char buf[TX_IT_BUF_SIZE];
    int len = snprintf(buf, sizeof(buf),
        "MTR q0=%lu/%lu/%lu q1=%lu/%lu/%lu q2=%lu/%lu/%lu q3=%lu/%lu/%lu "
        "e=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu u=%lX/%lX/%lX/%lX "
        "d=%u%u%u:%u,%u%u%u:%u,%u%u%u:%u,%u%u%u:%u\r\n",
        (unsigned long)(stats[0].tx_count[0] + stats[0].tx_count[1]),
        (unsigned long)(stats[0].rx_count[0] + stats[0].rx_count[1]),
        (unsigned long)(stats[0].miss_count[0] + stats[0].miss_count[1]),
        (unsigned long)(stats[1].tx_count[0] + stats[1].tx_count[1]),
        (unsigned long)(stats[1].rx_count[0] + stats[1].rx_count[1]),
        (unsigned long)(stats[1].miss_count[0] + stats[1].miss_count[1]),
        (unsigned long)(stats[2].tx_count[0] + stats[2].tx_count[1]),
        (unsigned long)(stats[2].rx_count[0] + stats[2].rx_count[1]),
        (unsigned long)(stats[2].miss_count[0] + stats[2].miss_count[1]),
        (unsigned long)(stats[3].tx_count[0] + stats[3].tx_count[1]),
        (unsigned long)(stats[3].rx_count[0] + stats[3].rx_count[1]),
        (unsigned long)(stats[3].miss_count[0] + stats[3].miss_count[1]),
        (unsigned long)busy,
        (unsigned long)txe,
        (unsigned long)crc,
        (unsigned long)id,
        (unsigned long)resync,
        (unsigned long)uart,
        (unsigned long)rst,
        (unsigned long)stats[0].schedule_overrun_count,
        (unsigned long)stats[0].uart_error_bits,
        (unsigned long)stats[1].uart_error_bits,
        (unsigned long)stats[2].uart_error_bits,
        (unsigned long)stats[3].uart_error_bits,
        stats[0].rx_dma_enabled, stats[0].rx_dma_circular,
        stats[0].uart_rx_dma_enabled, stats[0].rx_dma_remaining,
        stats[1].rx_dma_enabled, stats[1].rx_dma_circular,
        stats[1].uart_rx_dma_enabled, stats[1].rx_dma_remaining,
        stats[2].rx_dma_enabled, stats[2].rx_dma_circular,
        stats[2].uart_rx_dma_enabled, stats[2].rx_dma_remaining,
        stats[3].rx_dma_enabled, stats[3].rx_dma_circular,
        stats[3].uart_rx_dma_enabled, stats[3].rx_dma_remaining);

    if (len > 0 && len < (int)sizeof(buf) && comm_ctx.uart != NULL) {
        (void)HAL_UART_Transmit(comm_ctx.uart, (uint8_t *)buf, (uint16_t)len, 100U);
        comm_ctx.tx_busy = 0U;
    }
}
