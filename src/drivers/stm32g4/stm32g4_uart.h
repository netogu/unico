#pragma once
#ifndef STM32G4_UART_H
#define STM32G4_UART_H

#include "unilib.h"
#include <stdint.h>

#define UART_TX_BUFFER_SIZE 1024
#define UART_RX_BUFFER_SIZE 512

typedef struct uart_s {
  void *uart_base;
  ul_fifo_t rx_fifo;
  ul_fifo_t tx_fifo;
  size_t dma_tx_pending;
} uart_t;

typedef struct uart_config_s {
  uint32_t baudrate;
  enum uart_word_len_e {
    UART_DATA_BITS_8 = 0,
    UART_DATA_BITS_9 = 1,
    UART_DATA_BITS_7 = 2,
  } word_length;
  enum uart_stop_bits_e {
    UART_STOP_BITS_1 = 0,
    UART_STOP_BITS_2 = 2,
  } stop_bits;
  enum uart_parity_e {
    UART_PARITY_EVEN = 0,
    UART_PARITY_ODD = 1,
    UART_PARITY_NONE = 2,
  } parity;
  enum uart_mode_e {
    UART_MODE_RX = 0,
    UART_MODE_TX = 1,
    UART_MODE_RX_TX = 2,
  } mode;
  enum uart_flow_control_e {
    UART_FLOW_CONTROL_NONE = 0,
    UART_FLOW_CONTROL_RTS = 1,
    UART_FLOW_CONTROL_CTS = 2,
    UART_FLOW_CONTROL_RTS_CTS = 3,
  } flow_control;
} uart_config_t;

void uart_init(uart_t *self, uart_config_t config);
void uart_init_dma(uart_t *self, uart_config_t config);
size_t uart_write(uart_t *self, uint8_t *data, size_t size);
size_t uart_read(uart_t *self, uint8_t *data, size_t size);
int uart_dma_transfer(uart_t *self);
void uart_dma_receive(uart_t *self);
int uart_fifo_update_on_dma(uart_t *self);

#endif // STM32G4_UART_H
