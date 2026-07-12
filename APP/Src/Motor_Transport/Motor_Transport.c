#include "Motor_Transport.h"

#include "crc_ccitt.h"

#include <string.h>

#define MOTOR_TRANSPORT_CHANNEL_COUNT 4U
#define MOTOR_TRANSPORT_MOTOR_COUNT 2U
#define MOTOR_TRANSPORT_RING_SIZE 256U
#define MOTOR_TRANSPORT_NO_PENDING 0xFFU
#define MOTOR_TRANSPORT_OFFLINE_TIMEOUT_MS 100U
#define MOTOR_TRANSPORT_QUIESCE_TIMEOUT_MS 5U

/* Extended-duration RF cascade plan is dry-run only pending review. */
#define MOTOR_TRANSPORT_ZERO_OUTPUT_ONLY 1U

_Static_assert((MOTOR_TRANSPORT_RING_SIZE & (MOTOR_TRANSPORT_RING_SIZE - 1U)) == 0U,
               "motor RX ring size must be a power of two");
_Static_assert(sizeof(ControlData_t) == 17U, "unexpected motor command frame size");
_Static_assert(sizeof(MotorData_t) == 16U, "unexpected motor feedback frame size");

typedef struct __attribute__((aligned(32)))
{
  uint8_t rx_ring[MOTOR_TRANSPORT_RING_SIZE];
  ControlData_t tx_frame[MOTOR_TRANSPORT_MOTOR_COUNT];
  UART_HandleTypeDef *uart;
  DMA_HandleTypeDef *rx_dma;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t leg_index;
  volatile uint8_t tx_busy;
  volatile uint8_t restart_rx;
  uint16_t read_index;
  uint8_t pending_motor;
  uint8_t feedback_online[MOTOR_TRANSPORT_MOTOR_COUNT];
  uint32_t last_feedback_tick[MOTOR_TRANSPORT_MOTOR_COUNT];
  uint32_t tx_count[MOTOR_TRANSPORT_MOTOR_COUNT];
  uint32_t rx_count[MOTOR_TRANSPORT_MOTOR_COUNT];
  uint32_t miss_count[MOTOR_TRANSPORT_MOTOR_COUNT];
  uint32_t busy_count;
  uint32_t tx_error_count;
  uint32_t crc_error_count;
  uint32_t id_error_count;
  uint32_t resync_count;
  uint32_t uart_error_count;
  uint32_t uart_error_bits;
  uint32_t restart_count;
} Motor_TransportChannel;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
/* RX rings and TX frames are DMA-visible; this NOLOAD object is cleared in Init. */
static Motor_TransportChannel channels[MOTOR_TRANSPORT_CHANNEL_COUNT]
    __attribute__((section(".dma_buffer"), aligned(32)));
static volatile uint8_t transport_initialized;
static volatile uint8_t transport_running;
static volatile uint32_t transport_ticks;
static uint32_t schedule_overrun_count;
static Motor_TransportCallbacks transport_callbacks;

static HAL_StatusTypeDef configure_rx_mode(Motor_TransportChannel *channel, uint32_t mode)
{
  if (channel == NULL || channel->rx_dma == NULL) return HAL_ERROR;
  channel->rx_dma->Init.Mode = mode;
  return HAL_DMA_Init(channel->rx_dma);
}

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

static uint8_t rx_ring_is_armed(const Motor_TransportChannel *channel)
{
  DMA_Stream_TypeDef *stream = (DMA_Stream_TypeDef *)channel->rx_dma->Instance;
  return ((stream->CR & (DMA_SxCR_EN | DMA_SxCR_CIRC)) ==
          (DMA_SxCR_EN | DMA_SxCR_CIRC)) &&
         ((channel->uart->Instance->CR3 & USART_CR3_DMAR) != 0U);
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
    /* Keep TC enabled: HAL_DMA_Abort_IT uses the DMA IRQ to finish a UART
       error abort and eventually call HAL_UART_ErrorCallback. */
    __HAL_DMA_ENABLE_IT(channel->rx_dma, DMA_IT_TC);
  }
  return result;
}

static void prepare_tx_frame(Motor_TransportChannel *channel, uint8_t motor)
{
  MOTOR_send command;
  memset(&command, 0, sizeof(command));
  if (transport_callbacks.load_command != NULL)
    (void)transport_callbacks.load_command(channel->leg_index, motor, &command);

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
  if (!transport_running) return;
  if (channel->tx_busy) {
    ++channel->busy_count;
    return;
  }

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
    ++channel->tx_error_count;
    return;
  }
  ++channel->tx_count[motor];
}

static void accept_feedback(Motor_TransportChannel *channel, const MotorData_t *frame)
{
  uint8_t motor = channel->pending_motor;
  MOTOR_recv feedback;

  memset(&feedback, 0, sizeof(feedback));
  memcpy(&feedback.motor_recv_data, frame, sizeof(*frame));
  feedback.rx_len = sizeof(*frame);
  if (!extract_data(&feedback) || feedback.motor_id != motor) return;

  uint32_t now = HAL_GetTick();
  channel->last_feedback_tick[motor] = now;
  channel->feedback_online[motor] = 1U;
  ++channel->rx_count[motor];
  channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
  if (transport_callbacks.feedback_received != NULL)
    transport_callbacks.feedback_received(channel->leg_index, motor, &feedback, now);

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
      ++channel->resync_count;
      continue;
    }
    if (ring_available(channel, write) < sizeof(MotorData_t)) return;

    MotorData_t frame;
    for (uint16_t i = 0U; i < sizeof(frame); ++i)
      ((uint8_t *)&frame)[i] = channel->rx_ring[(channel->read_index + i) &
                                                 (MOTOR_TRANSPORT_RING_SIZE - 1U)];
    channel->read_index = (uint16_t)((channel->read_index + sizeof(frame)) &
                                     (MOTOR_TRANSPORT_RING_SIZE - 1U));

    if (frame.CRC16 != crc_ccitt(0, (uint8_t *)&frame, 14U)) {
      ++channel->crc_error_count;
      continue;
    }
    if (channel->pending_motor == MOTOR_TRANSPORT_NO_PENDING ||
        frame.mode.id != channel->pending_motor) {
      ++channel->id_error_count;
      continue;
    }
    accept_feedback(channel, &frame);
    write = ring_write_index(channel);
  }
}

static void update_online_timeouts(Motor_TransportChannel *channel, uint32_t now)
{
  for (uint8_t motor = 0U; motor < MOTOR_TRANSPORT_MOTOR_COUNT; ++motor) {
    if ((now - channel->last_feedback_tick[motor]) <= MOTOR_TRANSPORT_OFFLINE_TIMEOUT_MS)
      continue;
    if (!channel->feedback_online[motor]) continue;
    channel->feedback_online[motor] = 0U;
    if (transport_callbacks.feedback_timeout != NULL)
      transport_callbacks.feedback_timeout(channel->leg_index, motor, now,
                                           now - channel->last_feedback_tick[motor]);
  }
}

HAL_StatusTypeDef Motor_Transport_Init(const Motor_TransportCallbacks *callbacks)
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

  transport_initialized = 0U;
  transport_running = 0U;
  transport_ticks = 0U;
  schedule_overrun_count = 0U;
  memset(&transport_callbacks, 0, sizeof(transport_callbacks));
  if (callbacks == NULL || callbacks->load_command == NULL ||
      callbacks->feedback_received == NULL || callbacks->feedback_timeout == NULL)
    return HAL_ERROR;
  transport_callbacks = *callbacks;
  memset(channels, 0, sizeof(channels));
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    channels[i] = setup[i];
    channels[i].pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    HAL_GPIO_WritePin(channels[i].de_port, channels[i].de_pin, GPIO_PIN_RESET);
    if (channels[i].uart == NULL || channels[i].rx_dma == NULL) return HAL_ERROR;
  }
  transport_initialized = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Motor_Transport_Stop(void)
{
  if (!transport_initialized) return HAL_OK;

  HAL_StatusTypeDef result = HAL_OK;
  transport_running = 0U;
  transport_ticks = 0U;

  /* Stop scheduling first, then let every active UART reach its TC callback
     before releasing DE or aborting the permanent RX rings. */
  uint32_t quiesce_start = HAL_GetTick();
  uint8_t any_tx_busy;
  do {
    any_tx_busy = 0U;
    for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i)
      any_tx_busy |= channels[i].tx_busy;
  } while (any_tx_busy &&
           (HAL_GetTick() - quiesce_start) < MOTOR_TRANSPORT_QUIESCE_TIMEOUT_MS);

  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (channel->tx_busy) {
      (void)HAL_UART_AbortTransmit(channel->uart);
      result = HAL_TIMEOUT;
    }
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    (void)HAL_UART_Abort(channel->uart);
    __HAL_UART_CLEAR_FLAG(channel->uart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    SET_BIT(channel->uart->Instance->RQR, UART_RXDATA_FLUSH_REQUEST);
    channel->tx_busy = 0U;
    channel->restart_rx = 0U;
    channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    channel->read_index = 0U;
    memset(channel->rx_ring, 0, sizeof(channel->rx_ring));
    if (configure_rx_mode(channel, DMA_NORMAL) != HAL_OK) result = HAL_ERROR;
  }
  return result;
}

HAL_StatusTypeDef Motor_Transport_Start(void)
{
  if (!transport_initialized) return HAL_ERROR;

  transport_running = 0U;
  transport_ticks = 0U;
  uint32_t now = HAL_GetTick();
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    (void)HAL_UART_Abort(channel->uart);
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    channel->tx_busy = 0U;
    channel->restart_rx = 0U;
    channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    channel->last_feedback_tick[0] = now;
    channel->last_feedback_tick[1] = now;
    channel->feedback_online[0] = 1U;
    channel->feedback_online[1] = 1U;
    if (configure_rx_mode(channel, DMA_CIRCULAR) != HAL_OK ||
        arm_rx_ring(channel) != HAL_OK) {
      (void)Motor_Transport_Stop();
      return HAL_ERROR;
    }
  }
  transport_running = 1U;
  return HAL_OK;
}

void Motor_Transport_Tick(void)
{
  if (transport_running) ++transport_ticks;
}

uint8_t Motor_Transport_IsZeroOutputOnly(void)
{
  return MOTOR_TRANSPORT_ZERO_OUTPUT_ONLY ? 1U : 0U;
}

void Motor_Transport_Service(void)
{
  if (!transport_running) return;

  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (!channel->restart_rx && !rx_ring_is_armed(channel)) {
      channel->restart_rx = 1U;
      channel->uart_error_bits |= channel->uart->ErrorCode;
      ++channel->uart_error_count;
    }
    if (channel->restart_rx) {
      channel->restart_rx = 0U;
      (void)HAL_UART_AbortReceive(channel->uart);
      __HAL_UART_CLEAR_FLAG(channel->uart,
                            UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
      SET_BIT(channel->uart->Instance->RQR, UART_RXDATA_FLUSH_REQUEST);
      if (configure_rx_mode(channel, DMA_CIRCULAR) == HAL_OK &&
          arm_rx_ring(channel) == HAL_OK) {
        ++channel->restart_count;
      }
    }
    parse_rx_ring(channel);
    /* accept_feedback() timestamps with HAL_GetTick().  Refresh now after
       parsing so a SysTick edge cannot make now older than last_feedback_tick
       and turn a fresh frame into an unsigned-underflow timeout. */
    update_online_timeouts(channel, HAL_GetTick());
  }

  uint32_t ticks;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  ticks = transport_ticks;
  transport_ticks = 0U;
  if (primask == 0U) __enable_irq();
  if (ticks == 0U) return;
  if (ticks > 1U) schedule_overrun_count += ticks - 1U;

  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (channel->pending_motor != MOTOR_TRANSPORT_NO_PENDING) {
      ++channel->miss_count[channel->pending_motor];
      channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    }
    send_motor(channel, 0U);
  }
}

uint8_t Motor_Transport_GetStats(uint8_t channel_index, Motor_TransportStats *stats)
{
  if (channel_index >= MOTOR_TRANSPORT_CHANNEL_COUNT || stats == NULL) return 0U;

  Motor_TransportChannel *channel = &channels[channel_index];
  DMA_Stream_TypeDef *rx_stream = (DMA_Stream_TypeDef *)channel->rx_dma->Instance;
  memset(stats, 0, sizeof(*stats));
  stats->running = transport_running;
  stats->leg_index = channel->leg_index;
  stats->pending_motor = channel->pending_motor;
  stats->tx_busy = channel->tx_busy;
  stats->rx_dma_enabled =
      (rx_stream->CR & DMA_SxCR_EN) != 0U ? 1U : 0U;
  stats->rx_dma_circular =
      (rx_stream->CR & DMA_SxCR_CIRC) != 0U ? 1U : 0U;
  stats->uart_rx_dma_enabled =
      (channel->uart->Instance->CR3 & USART_CR3_DMAR) != 0U ? 1U : 0U;
  stats->rx_dma_remaining = (uint16_t)__HAL_DMA_GET_COUNTER(channel->rx_dma);
  stats->rx_write_index = transport_running ? ring_write_index(channel) : 0U;
  stats->rx_read_index = channel->read_index;
  stats->tx_count[0] = channel->tx_count[0];
  stats->tx_count[1] = channel->tx_count[1];
  stats->rx_count[0] = channel->rx_count[0];
  stats->rx_count[1] = channel->rx_count[1];
  stats->miss_count[0] = channel->miss_count[0];
  stats->miss_count[1] = channel->miss_count[1];
  stats->busy_count = channel->busy_count;
  stats->tx_error_count = channel->tx_error_count;
  stats->crc_error_count = channel->crc_error_count;
  stats->id_error_count = channel->id_error_count;
  stats->resync_count = channel->resync_count;
  stats->uart_error_count = channel->uart_error_count;
  stats->uart_error_bits = channel->uart_error_bits;
  stats->restart_count = channel->restart_count;
  stats->schedule_overrun_count = schedule_overrun_count;
  return 1U;
}

uint8_t Motor_Transport_HandleTxComplete(UART_HandleTypeDef *huart)
{
  if (!transport_initialized) return 0U;
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
  if (!transport_initialized) return 0U;
  for (uint8_t i = 0U; i < MOTOR_TRANSPORT_CHANNEL_COUNT; ++i) {
    Motor_TransportChannel *channel = &channels[i];
    if (huart != channel->uart) continue;
    HAL_GPIO_WritePin(channel->de_port, channel->de_pin, GPIO_PIN_RESET);
    channel->tx_busy = 0U;
    channel->pending_motor = MOTOR_TRANSPORT_NO_PENDING;
    channel->restart_rx = transport_running;
    channel->uart_error_bits |= huart->ErrorCode;
    ++channel->uart_error_count;
    if (transport_callbacks.uart_error != NULL)
      transport_callbacks.uart_error(channel->leg_index, huart->ErrorCode,
                                     HAL_GetTick());
    return 1U;
  }
  return 0U;
}
