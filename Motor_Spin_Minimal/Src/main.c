#include "stm32h7xx_hal.h"

#include <stdio.h>
#include <string.h>
#include "GO-M8010-6.h"

#define RS485_DE_PORT          GPIOA
#define RS485_DE_PIN           GPIO_PIN_4
#define TEST_SPEED_RAD_S       0.5f
#define TEST_SPEED_KW          0.10f
#define ACTIVE_SEND_PERIOD_MS  50U
#define ACTIVE_PRINT_PERIOD_MS 500U

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

static void print_line(const char *text)
{
  if (text)
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100);
}

static void print_menu(void)
{
  print_line("\r\n=== Minimal Left-Front Motor Test ===\r\n");
  print_line("No ESP32. No full robot control. UART1 debug only.\r\n");
  print_line("Motor bus: USART2 PA2/PA3, RS485 DE PA4, 4 Mbps.\r\n");
  print_line("Expected motor IDs on this bus: 0 and 1.\r\n");
  print_line("Commands:\r\n");
  print_line("  p: ping ID0 and ID1 with zero command\r\n");
  print_line("  a: spin ID0 +0.5 rad/s\r\n");
  print_line("  z: spin ID0 -0.5 rad/s\r\n");
  print_line("  k: spin ID1 +0.5 rad/s\r\n");
  print_line("  m: spin ID1 -0.5 rad/s\r\n");
  print_line("  s: stop both motors\r\n");
  print_line("Motors do not spin until a/z/k/m is received.\r\n\r\n");
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
  return SERVO_Send_recv(&cmd, fbk, RS485_DE_PORT, RS485_DE_PIN, &huart2);
}

static void print_result(uint8_t id, float speed, HAL_StatusTypeDef ret, MOTOR_recv *fbk)
{
  char buf[176];
  if (ret == HAL_OK && fbk->correct)
  {
    snprintf(buf, sizeof(buf),
             "ID%u cmdW=%+.2f -> OK fbk_id=%u pos=%.4f W=%.4f T=%.4f temp=%d err=%u\r\n",
             id, speed, fbk->motor_id, fbk->Pos, fbk->W, fbk->T, fbk->Temp, fbk->MError);
  }
  else
  {
    const char *reason = (ret == HAL_TIMEOUT) ? "TIMEOUT" : "UART/FRAME_ERROR";
    snprintf(buf, sizeof(buf),
             "ID%u cmdW=%+.2f -> %s ret=%d correct=%d rx_len=%u\r\n",
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
  print_menu();
  ping_bus();

  int active_id = -1;
  float active_speed = 0.0f;
  uint32_t last_send = 0;
  uint32_t last_print = 0;

  while (1)
  {
    uint8_t ch;
    if (HAL_UART_Receive(&huart1, &ch, 1, 5) == HAL_OK)
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

    if (active_id >= 0 && (HAL_GetTick() - last_send) >= ACTIVE_SEND_PERIOD_MS)
    {
      MOTOR_recv fbk;
      HAL_StatusTypeDef ret;
      last_send = HAL_GetTick();
      ret = send_motor((uint8_t)active_id, active_speed, &fbk);
      if ((HAL_GetTick() - last_print) >= ACTIVE_PRINT_PERIOD_MS)
      {
        last_print = HAL_GetTick();
        print_result((uint8_t)active_id, active_speed, ret, &fbk);
      }
    }
  }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = RS485_DE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_PORT, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
  HAL_UARTEx_DisableFifoMode(&huart1);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 4000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
    Error_Handler();
  HAL_UARTEx_DisableFifoMode(&huart2);
}

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if (huart->Instance == USART1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
      Error_Handler();

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (huart->Instance == USART2)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
      Error_Handler();

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
