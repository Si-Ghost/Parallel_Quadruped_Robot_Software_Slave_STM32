#ifndef PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_TRANSPORT_H
#define PARALLEL_QUADRUPED_ROBOT_STM32_MOTOR_TRANSPORT_H

#include "main.h"
#include "GO-M8010-6.h"

typedef uint8_t (*Motor_TransportLoadCommandFn)(uint8_t leg,
                                                uint8_t motor,
                                                MOTOR_send *command);
typedef void (*Motor_TransportFeedbackFn)(uint8_t leg,
                                          uint8_t motor,
                                          const MOTOR_recv *feedback,
                                          uint32_t timestamp);
typedef void (*Motor_TransportOfflineFn)(uint8_t leg,
                                         uint8_t motor,
                                         uint32_t timestamp,
                                         uint32_t feedback_age_ms);
typedef void (*Motor_TransportUartErrorFn)(uint8_t leg,
                                           uint32_t error_bits,
                                           uint32_t timestamp);

typedef struct
{
  Motor_TransportLoadCommandFn load_command;
  Motor_TransportFeedbackFn feedback_received;
  Motor_TransportOfflineFn feedback_timeout;
  Motor_TransportUartErrorFn uart_error;
} Motor_TransportCallbacks;

/*
 * Owns UART2/UART8/UART7/UART5 only while running.
 * Init must be called before Stop/Start because the DMA state lives in a
 * NOLOAD RAM_D2 section.  Stop restores normal RX DMA for blocking handshakes;
 * Start switches RX DMA to circular and arms all four permanent rings.
 */
HAL_StatusTypeDef Motor_Transport_Init(const Motor_TransportCallbacks *callbacks);
HAL_StatusTypeDef Motor_Transport_Start(void);
HAL_StatusTypeDef Motor_Transport_Stop(void);
void Motor_Transport_Tick(void);
void Motor_Transport_Service(void);

typedef struct
{
  uint8_t running;
  uint8_t leg_index;
  uint8_t pending_motor;
  uint8_t tx_busy;
  uint8_t rx_dma_enabled;
  uint8_t rx_dma_circular;
  uint8_t uart_rx_dma_enabled;
  uint16_t rx_write_index;
  uint16_t rx_read_index;
  uint16_t rx_dma_remaining;
  uint32_t tx_count[2];
  uint32_t rx_count[2];
  uint32_t miss_count[2];
  uint32_t busy_count;
  uint32_t tx_error_count;
  uint32_t crc_error_count;
  uint32_t id_error_count;
  uint32_t resync_count;
  uint32_t uart_error_count;
  uint32_t uart_error_bits;
  uint32_t restart_count;
  uint32_t schedule_overrun_count;
} Motor_TransportStats;

uint8_t Motor_Transport_GetStats(uint8_t channel, Motor_TransportStats *stats);

/* Return non-zero only when the UART belongs to this transport. */
uint8_t Motor_Transport_HandleTxComplete(UART_HandleTypeDef *huart);
uint8_t Motor_Transport_HandleError(UART_HandleTypeDef *huart);

#endif
