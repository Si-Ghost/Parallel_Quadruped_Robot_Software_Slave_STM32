#include "Motor_Transport.h"

#include "GO-M8010-6.h"
#include "Leg_Control.h"
#include "crc_ccitt.h"

#include <string.h>

#define MOTOR_TRANSPORT_CHANNEL_COUNT 4U
#define MOTOR_TRANSPORT_MOTORS_PER_CHANNEL 2U
#define MOTOR_TRANSPORT_RING_SIZE 256U
#define MOTOR_TRANSPORT_NO_PENDING 0xFFU

/*
 * Safety gate for the first DMA migration.  The Leg_Control command model is
 * intentionally left intact, but only FOC frames with all control outputs at
 * zero reach the motors.  Remove this gate only under a separate, explicitly
 * authorized motion-validation change.
 */
#define MOTOR_TRANSPORT_ZERO_OUTPUT_ONLY 1U

typedef struct __attribute__((aligned(32)))
{
  uint8_t rx_ring[MOTOR_TRANSPORT_RING_SIZE];
  ControlData_t tx_frame[MOTOR_TRANSPORT_MOTORS_PER_CHANNEL];
  UART_HandleTypeDef *uart;
  DMA_HandleTypeDef *rx_dma;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t leg_index;
  volatile uint8_t tx_busy;
  volatile uint8_t restart_rx;
  uint16_t read_index;
  uint8_t pending_motor;
} Motor_TransportChannel;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern Leg_HandlerTypeDef *Legs[4];

static Motor_TransportChannel channels[MOTOR_TRANSPORT_CHANNEL_COUNT]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint8_t transport_running;
static volatile uint32_t transport_ticks;

static uint16_t ring_write_index(const Motor_TransportChannel *channel)
{
  uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(channel->rx_dma);
  if (remain > MOTOR_TRANSPORT_RING_SIZE) return channel->read_index;
  return (uint16_t)((MOTOR_TRANSPORT_RING_SIZE - remain) &
                    (MOTOR_TRANSPORT_RING_SIZE - 1U));
}

static uint16_t ring_available(const Motor_TransportChannel *channel, uint16_t write)
{
  return (uint16_t)((write - channel->read_index) &
                    (MOTOR_TRANSPORT_RING_SIZE - 1U));
}

static HAL_StatusTypeDef arm_rx_ring(Motor_TransportChannel *channel)
{
  memset(channel->rx_ring, 0, sizeof(channel->rx_ring));
  channel->read_index = 0U;
  HAL_StatusTypeDef result = HAL_UART_Receive_DMA(channel->uart,
                                                    channel->rx_ring,
                                                    sizeof(channel->rx_ring));
  if (result == HAL_OK) {
    __HAL_DMA_DISABLE_IT(channel->rx_dma, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(channel->rx_dma, DMA_IT_TC);
  }
  return result;
}

static void prepare_tx_frame(Motor_TransportChannel *channel, uint8_t motor)
{
  MOTOR_send command = Legs[channel->leg_index]->motor_cmd[motor];

#if MOTOR_TRANSPORT_ZERO_OUTPUT_ONLY
  command.mode = 1U;
  command.T = 0.0f;
  command.W = 0.0f;
  command.Pos = 0.0f;
  command.K_P = 0.0f;
  command.K_W = 0.0f;
#endif

  command.id = motor;
  modify_data(&command);
  memcpy(&channel->tx_frame[motor], &command.motor_send_data,
         sizeof(channel->tx_frame[motor]));
}

static void send_motor(Motor_TransportChannel *channel, uint8_t motor)
{
  if (!transport_running || channel->tx_busy) return;

  channel->read_index = ring_write_index(channel);
  channel->pending_motor = motor;
  channel->tx_busy = 1U;
  prepare_tx_frame(channel, motor);
  HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_SET);

  if (HAL_UART_Transmit_DMA(channel->uart, (uint8_t *)&channel->tx_frame[motor],
                            sizeof(channel->tx_frame[motor])) != HAL_OK) {
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    channel->tx_busy = 0U;
    channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
  }
}

static void accept_feedback(Motor_TransportChannel *channel, const MotorData_t *frame)
{
  uint8_t motor = channel->pending_motor;
  MOTOR_recv *feedback = &Legs[channel->leg_index]->motor_data[motor];
  Motor_RuntimeStateTypeDef *state = &Legs[channel->leg_index]->motor_state[motor];

  memcpy(&feedback->motor_recv_data, frame, sizeof(*frame));
  feedback->rx_len = sizeof(*frame);
  if (!extract_data(feedback) || feedback->motor_id != motor) return;

  state->angle = feedback->Pos;
  state->speed = feedback->W;
  state->angle_valid = Motor_Angle_Valid;
  state->io_error_count = 0U;
  channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;

  if (motor == 0U) send_motor(channel, 1U);
}

static void parse_rx_ring(Motor_TransportChannel *channel)
{
  uint16_t write = ring_write_index(channel);
  while (ring_available(channel, write) >= 2U) {
    uint8_t h0 = channel->rx_ring[channel->read_index & (MOTOR_TRANSPORT_RING_SIZE - 1U)];
    uint8_t h1 = channel->rx_ring[(channel->read_index + 1U) &
                                  (MOTOR_TRANSPORT_RING_SIZE - 1U)];
    if ((h0 != 0xFD && h0 != 0xFE) || h1 != 0xEE) {
      channel->read_index = (uint16_t)((channel->read_index + 1U) &
                                       (MOTOR_TRANSPORT_RING_SIZE - 1U));
      continue;
    }
    if (ring_available(channel, write) < sizeof(MotorData_t)) return;

    MotorData_t frame;
    for (uint16_t i = 0U; i < sizeof(frame); ++i)
      ((uint8_t *)&frame)[i] = channel->rx_ring[(channel->read_index + i) &
                                                 (MOTOR_TRANSPORT_RING_SIZE - 1U)];
    channel->read_index = (uint16_t)((channel->read_index + sizeof(frame)) &
                                     (MOTOR_TRANSPORT_RING_SIZE - 1U));

    if (frame.CRC16 != crc_ccitt(0, (uint8_t *)&frame, 14U)) continue;
    if (channel->pending_motor == MOTOR_TRANSPORT_NO_PENDING ||
        frame.mode.id != channel->pending_motor) continue;
    accept_feedback(channel, &frame);
    write = ring_write_index(channel);
  }
}

void Motor_Transport_Start(void)
{
  Motor_TransportChannel setup[MOTOR_TRANSPORT_CHANNEL_COUNT] = {
      {.uart = &huart2, .rx_dma = &hdma_usart2_rx,
       .de_port = Left_Front_Leg_Control_GPIO_Port, .de_pin = Left_Front_Leg_Control_Pin,
       .leg_index = 0U},
      {.uart = &huart8, .rx_dma = &hdma_uart8_rx,
       .de_port = Right_Front_Leg_Control_GPIO_Port, .de_pin = Right_Front_Leg_Control_Pin,
       .leg_index = 1U},
      {.uart = &huart7, .rx_dma = &hdma_uart7_rx,
       .de_port = Left_Back_Leg_Control_GPIO_Port, .de_pin = Left_Back_Leg_Control_Pin,
       .leg_index = 2U},
      {.uart = &huart5, .rx_dma = &hdma_uart5_rx,
       .de_port = Right_Back_Leg_Control_GPIO_Port, .de_pin = Right_Back_Leg_Control_Pin,
       .leg_index = 3U},
  };

  transport_running = 0U;
  transport_ticks = 0U;
  memset(channels, 0, sizeof(channels));
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    channels[i] = setup[i];
    channels[i].pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    HAL_GPIO_WritePin(channels[i].de_port, channels[i].de_pin, GPIO_PIN_RESET);
    if (arm_rx_ring(&channels[i]) != HAL_OK) Error_Handler();
  }
  transport_running = 1U;
}

void Motor_Transport_Stop(void)
{
  transport_running = 0U;
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (channel->uart == NULL) continue;
    channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    channel->tx_busy = 0U;
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    HAL_UART_Abort(channel->uart);
    __HAL_UART_CLEAR_FLAG(channel->uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
  }
}

void Motor_Transport_Tick(void)
{
  if (transport_running) ++transport_ticks;
}

void Motor_Transport_Service(void)
{
  if (!transport_running) return;

  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (channel->restart_rx) {
      channel->restart_rx = 0U;
      HAL_UART_AbortReceive(channel->uart);
      __HAL_UART_CLEAR_FLAG(channel->uart,
                            UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
      (void)arm_rx_ring(channel);
    }
    parse_rx_ring(channel);
  }

  uint32_t ticks;
  __disable_irq();
  ticks = transport_ticks;
  transport_ticks = 0U;
  __enable_irq();
  if (ticks == 0U) return;

  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (channel->pending_motor != MOTOR_TRANSPORT_NO_PENDING)
      channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    send_motor(channel, 0U);
  }
}

uint8_t Motor_Transport_HandleTxComplete(UART_HandleTypeDef *huart)
{
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (huart != channel->uart) continue;
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    channel->tx_busy = 0U;
    return 1U;
  }
  return 0U;
}

uint8_t Motor_Transport_HandleError(UART_HandleTypeDef *huart)
{
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (huart != channel->uart) continue;
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    channel->tx_busy = 0U;
    channel->restart_rx = transport_running;
    return 1U;
  }
  return 0U;
}
