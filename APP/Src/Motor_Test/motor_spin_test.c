#include "motor_spin_test.h"

#include <stdio.h>
#include <string.h>
#include "GO-M8010-6.h"

#define TEST_SPEED_RAD_S     0.5f
#define TEST_SPEED_KW        0.10f
#define TEST_PERIOD_MS       50U
#define TEST_PRINT_MS        500U

static UART_HandleTypeDef *dbg_uart;
static UART_HandleTypeDef *mot_uart;
static GPIO_TypeDef *rs485_port;
static uint16_t rs485_pin;

static void print_line(const char *text)
{
  if (!dbg_uart || !text)
    return;
  HAL_UART_Transmit(dbg_uart, (uint8_t *)text, strlen(text), 100);
}

static void print_menu(void)
{
  print_line("\r\n=== Left Front Motor Spin Test ===\r\n");
  print_line("UART2: PA2 TX, PA3 RX, PA4 RS485 DE\r\n");
  print_line("Motor IDs on this bus: 0 and 1\r\n");
  print_line("Commands:\r\n");
  print_line("  p: ping ID0 and ID1 with zero command\r\n");
  print_line("  a: spin ID0 +0.5 rad/s\r\n");
  print_line("  z: spin ID0 -0.5 rad/s\r\n");
  print_line("  k: spin ID1 +0.5 rad/s\r\n");
  print_line("  m: spin ID1 -0.5 rad/s\r\n");
  print_line("  s: stop both motors\r\n");
  print_line("No motor spins until a/z/k/m is received.\r\n\r\n");
}

static void fill_cmd(MOTOR_send *cmd, uint8_t id, float speed)
{
  memset(cmd, 0, sizeof(*cmd));
  cmd->id = id;
  cmd->mode = 1;
  cmd->T = 0.0f;
  cmd->W = speed;
  cmd->Pos = 0.0f;
  cmd->K_P = 0.0f;
  cmd->K_W = (speed == 0.0f) ? 0.0f : TEST_SPEED_KW;
  modify_data(cmd);
}

static HAL_StatusTypeDef send_motor(uint8_t id, float speed, MOTOR_recv *fbk)
{
  MOTOR_send cmd;
  fill_cmd(&cmd, id, speed);
  memset(fbk, 0, sizeof(*fbk));
  return SERVO_Send_recv(&cmd, fbk, rs485_port, rs485_pin, mot_uart);
}

static void print_result(uint8_t id, float speed, HAL_StatusTypeDef ret, MOTOR_recv *fbk)
{
  char buf[160];
  if (ret == HAL_OK && fbk->correct)
  {
    snprintf(buf, sizeof(buf),
             "ID%u cmdW=%+.2f -> OK fbk_id=%u pos=%.4f W=%.4f T=%.4f temp=%d err=%u\r\n",
             id, speed, fbk->motor_id, fbk->Pos, fbk->W, fbk->T, fbk->Temp, fbk->MError);
  }
  else
  {
    const char *reason = (ret == HAL_TIMEOUT) ? "TIMEOUT" : "UART/FRAME_ERROR";
    snprintf(buf, sizeof(buf), "ID%u cmdW=%+.2f -> %s ret=%d correct=%d rx_len=%u\r\n",
             id, speed, reason, (int)ret, fbk->correct, (unsigned int)fbk->rx_len);
  }
  print_line(buf);
}

static void ping_bus(void)
{
  MOTOR_recv fbk;
  HAL_StatusTypeDef ret;

  ret = send_motor(0, 0.0f, &fbk);
  print_result(0, 0.0f, ret, &fbk);
  HAL_Delay(20);

  ret = send_motor(1, 0.0f, &fbk);
  print_result(1, 0.0f, ret, &fbk);
}

static void stop_all(void)
{
  MOTOR_recv fbk;
  HAL_StatusTypeDef ret;

  ret = send_motor(0, 0.0f, &fbk);
  print_result(0, 0.0f, ret, &fbk);
  HAL_Delay(20);

  ret = send_motor(1, 0.0f, &fbk);
  print_result(1, 0.0f, ret, &fbk);
}

void MotorSpinTest_Run(UART_HandleTypeDef *debug_uart,
                       UART_HandleTypeDef *motor_uart,
                       GPIO_TypeDef *dir_port,
                       uint16_t dir_pin)
{
  dbg_uart = debug_uart;
  mot_uart = motor_uart;
  rs485_port = dir_port;
  rs485_pin = dir_pin;

  HAL_GPIO_WritePin(rs485_port, rs485_pin, GPIO_PIN_RESET);
  print_menu();
  ping_bus();

  int active_id = -1;
  float active_speed = 0.0f;
  uint32_t last_send = 0;
  uint32_t last_print = 0;

  while (1)
  {
    uint8_t ch;
    if (HAL_UART_Receive(dbg_uart, &ch, 1, 5) == HAL_OK)
    {
      if (ch == 'p')
      {
        active_id = -1;
        active_speed = 0.0f;
        ping_bus();
      }
      else if (ch == 's')
      {
        active_id = -1;
        active_speed = 0.0f;
        stop_all();
      }
      else if (ch == 'a')
      {
        active_id = 0;
        active_speed = TEST_SPEED_RAD_S;
        print_line("ID0 speed test +0.5 rad/s\r\n");
      }
      else if (ch == 'z')
      {
        active_id = 0;
        active_speed = -TEST_SPEED_RAD_S;
        print_line("ID0 speed test -0.5 rad/s\r\n");
      }
      else if (ch == 'k')
      {
        active_id = 1;
        active_speed = TEST_SPEED_RAD_S;
        print_line("ID1 speed test +0.5 rad/s\r\n");
      }
      else if (ch == 'm')
      {
        active_id = 1;
        active_speed = -TEST_SPEED_RAD_S;
        print_line("ID1 speed test -0.5 rad/s\r\n");
      }
    }

    if (active_id >= 0 && (HAL_GetTick() - last_send) >= TEST_PERIOD_MS)
    {
      MOTOR_recv fbk;
      HAL_StatusTypeDef ret;
      last_send = HAL_GetTick();
      ret = send_motor((uint8_t)active_id, active_speed, &fbk);
      if ((HAL_GetTick() - last_print) >= TEST_PRINT_MS)
      {
        last_print = HAL_GetTick();
        print_result((uint8_t)active_id, active_speed, ret, &fbk);
      }
    }
  }
}
