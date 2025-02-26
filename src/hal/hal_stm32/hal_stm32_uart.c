/******************************************************************************
 * File: uart.c
 * Description: Implementation of UART driver for STM32G4 series of
 *microcontrollers.
 *
 * Author: Ernesto Gonzalez Urdaneta
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "hal.h"
#include "stm32g474xx.h"

#define UART_STATUS_OK 0
#define UART_STATUS_ERROR 1
#define UART_STATUS_BUSY 2
#define UART_STATUS_TIMEOUT 3

#define UART_CLOCK_SOURCE_PCLK 0
#define UART_CLOCK_SOURCE_SYSCLK 1
#define UART_CLOCK_SOURCE_HSI 2
#define UART_CLOCK_SOURCE_LSE 3

#define UART_INPUT_CLOCK_PRESCALER_NONE 0
#define UART_INPUT_CLOCK_PRESCALER_DIV_2 1
#define UART_INPUT_CLOCK_PRESCALER_DIV_4 2
#define UART_INPUT_CLOCK_PRESCALER_DIV_6 3
#define UART_INPUT_CLOCK_PRESCALER_DIV_8 4
#define UART_INPUT_CLOCK_PRESCALER_DIV_10 5
#define UART_INPUT_CLOCK_PRESCALER_DIV_12 6
#define UART_INPUT_CLOCK_PRESCALER_DIV_16 7
#define UART_INPUT_CLOCK_PRESCALER_DIV_32 8
#define UART_INPUT_CLOCK_PRESCALER_DIV_64 9
#define UART_INPUT_CLOCK_PRESCALER_DIV_128 10
#define UART_INPUT_CLOCK_PRESCALER_DIV_256 11

#define USART1_DMA_RX_REQ_NUM 24
#define USART1_DMA_TX_REQ_NUM 25

#define USART3_DMA_RX_REQ_NUM 28
#define USART3_DMA_TX_REQ_NUM 29

#define LPUART1_DMA_RX_REQ_NUM 34
#define LPUART1_DMA_TX_REQ_NUM 35

#define UART_DMA_RX_BUFFER_SIZE 64
static uint8_t uart_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];

static inline void lpuart_init(hal_uart_t *self) {
  (void)self;

  // Set LPUART clock source to PCLK
  RCC->CCIPR &= ~(RCC_CCIPR_LPUART1SEL);
  RCC->CCIPR |= (UART_CLOCK_SOURCE_PCLK << RCC_CCIPR_LPUART1SEL_Pos);

  // Enable LPUART clock
  RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
  uint32_t tmpreg = RCC->APB1ENR2;
  (void)tmpreg;

  NVIC_EnableIRQ(LPUART1_IRQn);
}

static inline void usart1_init(hal_uart_t *self) {
  (void)self;
  // Set USART1 clock source to PCLK
  RCC->CCIPR &= ~(RCC_CCIPR_USART1SEL);
  RCC->CCIPR |= (UART_CLOCK_SOURCE_PCLK << RCC_CCIPR_USART1SEL_Pos);

  // Enable USART1 clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  uint32_t tmpreg = RCC->APB2ENR;
  (void)tmpreg;

  // Enable USART1 interrupt
  NVIC_EnableIRQ(USART1_IRQn);
}

static inline void usart3_init(hal_uart_t *self) {
  (void)self;
  // Set USART3 clock source to PCLK
  RCC->CCIPR &= ~(RCC_CCIPR_USART3SEL);
  RCC->CCIPR |= (UART_CLOCK_SOURCE_PCLK << RCC_CCIPR_USART3SEL_Pos);

  // Enable USART3 clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
  uint32_t tmpreg = RCC->APB1ENR1;
  (void)tmpreg;

  // Enable USART3 interrupt
  NVIC_EnableIRQ(USART3_IRQn);
}

static int hal_usart_enable_dma(hal_uart_t *self) {

  USART_TypeDef *uart = (USART_TypeDef *)self->port;

  // Turn on DMA1 and DMAMUX1 clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
  uint32_t tmpreg = RCC->AHB1ENR;
  (void)tmpreg;

  // Configure DMA1 Channel 2 for UART RX
  DMA1_Channel2->CPAR = (uint32_t)&(uart->RDR);
  DMA1_Channel2->CMAR = (uint32_t)uart_dma_rx_buffer;
  DMA1_Channel2->CNDTR = UART_DMA_RX_BUFFER_SIZE;
  DMA1_Channel2->CCR = 0;
  // Circular mode, enable transfer complete and half transfer interrupts
  DMA1_Channel2->CCR |=
      DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_HTIE;
  // Route USART3 RX DMA REQ to DMA1 Channel 2

  // Configure DMA1 Channel 3 for UART TX
  DMA1_Channel3->CPAR = (uint32_t)&(uart->TDR);
  DMA1_Channel3->CMAR = 0;
  DMA1_Channel3->CNDTR = 0;
  DMA1_Channel3->CCR = 0;
  // Enable memory increment and set direction to memory-to-peripheral
  DMA1_Channel3->CCR |=
      DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_HTIE;

  // Route DMA Requests
  if (uart == USART3) {
    DMAMUX1_Channel1->CCR = USART3_DMA_RX_REQ_NUM;
    DMAMUX1_Channel2->CCR = USART3_DMA_TX_REQ_NUM;
  } else if (uart == LPUART1) {
    DMAMUX1_Channel1->CCR = LPUART1_DMA_RX_REQ_NUM;
    DMAMUX1_Channel2->CCR = LPUART1_DMA_TX_REQ_NUM;
  } else if (uart == USART1) {
    DMAMUX1_Channel1->CCR = USART1_DMA_RX_REQ_NUM;
    DMAMUX1_Channel2->CCR = USART1_DMA_TX_REQ_NUM;
  } else {
    return -1; // error
  }

  // Enable DMA1 Channel 2
  DMA1_Channel2->CCR |= DMA_CCR_EN;
  // Enable DMA1 Channel 3
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  // Enable USART DMA
  uart->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);

  // Enable interrupts
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  return 0;
}

int hal_uart_start_dma_tx_transfer(hal_uart_t *self) {

  DMA_Channel_TypeDef *dma_channel = DMA1_Channel2;

  if (self->tx_dma_current_transfer_size > 0) {
    // DMA is busy
    return 1;
  }

  self->tx_dma_current_transfer_size =
      hal_uart_fifo_get_linear_size(&self->tx_fifo);

  if (self->tx_dma_current_transfer_size > 0) {
    // Some data to transfer

    // Disable Interrupts
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Disable the DMA channel
    dma_channel->CCR &= ~DMA_CCR_EN;

    DMA1->IFCR =
        DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3 | DMA_IFCR_CGIF3;

    // Set the memory address
    dma_channel->CMAR = (uint32_t)&(self->tx_fifo.buffer[self->tx_fifo.tail]);
    // Set the number of data items to transfer
    // dma_channel->CNDTR = self->tx_dma_current_transfer_size;
    dma_channel->CNDTR = self->tx_dma_current_transfer_size;
    // Enable the DMA channel
    dma_channel->CCR |= DMA_CCR_EN;
    // Re-enable interrupts
    __set_PRIMASK(primask);
  }

  return 0;
}

void hal_uart_service_rx_dma(hal_uart_t *self) {

  static uint32_t prev_buffer_index = 0;
  uint32_t buffer_index = UART_DMA_RX_BUFFER_SIZE - DMA1_Channel2->CNDTR;

  if (buffer_index != prev_buffer_index) {
    // Data has been received

    if (buffer_index > prev_buffer_index) {
      // Data is contiguous
      uint32_t bytes_read = buffer_index - prev_buffer_index;
      for (uint16_t i = 0; i < bytes_read; i++) {
        hal_uart_fifo_push(&self->rx_fifo,
                           uart_dma_rx_buffer[prev_buffer_index + i]);
      }
    } else {
      // Data is split
      uint32_t bytes_read = UART_DMA_RX_BUFFER_SIZE - prev_buffer_index;
      for (uint16_t i = 0; i < bytes_read; i++) {
        hal_uart_fifo_push(&self->rx_fifo,
                           uart_dma_rx_buffer[prev_buffer_index + i]);
      }
      for (uint16_t i = 0; i < buffer_index; i++) {
        hal_uart_fifo_push(&self->rx_fifo, uart_dma_rx_buffer[i]);
      }
    }
  }

  prev_buffer_index = buffer_index;

  if (prev_buffer_index == UART_DMA_RX_BUFFER_SIZE) {
    prev_buffer_index = 0;
  }
}

void hal_uart_clear_fifo(hal_uart_fifo_t *self) {
  self->head = 0;
  self->tail = 0;
  self->size = 0;
}

int hal_uart_init_dma(hal_uart_t *self, const hal_uart_config_t *config) {
  if (0 != hal_uart_init(self, config)) {
    return -1; // error
  }

  USART_TypeDef *uart = (USART_TypeDef *)self->port;

  self->tx_dma_current_transfer_size = 0;
  if (0 != hal_usart_enable_dma(self)) {
    return -1; // error
  }

  // Enable UART
  uart->CR1 |= (USART_CR1_UE);
  return 0;
}

int hal_uart_init(hal_uart_t *self, const hal_uart_config_t *config) {

  USART_TypeDef *uart = (USART_TypeDef *)self->port;

  hal_uart_clear_fifo(&self->rx_fifo);
  hal_uart_clear_fifo(&self->tx_fifo);

  if (uart == LPUART1) {
    lpuart_init(self);
  } else if (uart == USART3) {
    usart3_init(self);
  } else if (uart == USART1) {
    usart1_init(self);
  } else {
    return -1;
  }

  // Set UART Prescaler
  uart->PRESC = 0x00;
  uart->PRESC |= (UART_INPUT_CLOCK_PRESCALER_NONE & 0x0F);

  // Disable UART
  uart->CR1 &= ~(USART_CR1_UE);
  uart->CR1 = 0x00;

  if (uart == LPUART1) {
    // Set baudrate LPUART Specific
    uint32_t usartdiv = SystemCoreClock / config->baudrate * 256;
    uart->BRR = usartdiv & 0x0FFFFF; // 20 bits
  } else {
    // Set baudrate
    uint32_t usartdiv = SystemCoreClock / config->baudrate;
    uart->BRR = usartdiv & 0x0FFFF; // 16 bits
  }

  // Set data bits
  // 8 bits by default
  uart->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
  if (config->word_length == UART_DATA_BITS_9) {
    uart->CR1 |= (USART_CR1_M0);
  } else if (config->word_length == UART_DATA_BITS_7) {
    uart->CR1 |= (USART_CR1_M1);
  }

  // Set parity
  uart->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
  if (config->parity == UART_PARITY_EVEN) {
    uart->CR1 |= (USART_CR1_PCE);
    uart->CR1 |= (config->parity << USART_CR1_PS_Pos);
  } // else LPUART_PARITY_NONE

  // Set stop bits
  uart->CR2 &= ~(USART_CR2_STOP);
  uart->CR2 |= (config->stop_bits << USART_CR2_STOP_Pos);

  // Set Flow Control
  uart->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
  if (config->flow_control & UART_FLOW_CONTROL_RTS) {
    uart->CR3 |= (USART_CR3_RTSE);
  } else if (config->flow_control & UART_FLOW_CONTROL_CTS) {
    uart->CR3 |= (USART_CR3_CTSE);
  } else if (config->flow_control & UART_FLOW_CONTROL_RTS_CTS) {
    uart->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
  } // else LPUART_FLOW_CONTROL_NONE

  // Set mode
  uart->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);
  if (config->mode == UART_MODE_RX) {
    uart->CR1 |= (USART_CR1_RE);
  } else if (config->mode == UART_MODE_TX) {
    uart->CR1 |= (USART_CR1_TE);
  } else if (config->mode == UART_MODE_RX_TX) {
    uart->CR1 |= (USART_CR1_RE | USART_CR1_TE);
  } // else LPUART_MODE_NONE

  // Enable UART RXNE interrupt
  // uart->CR1 |= (USART_CR1_RXNEIE);
  // Enable UART TXE interrupt
  // uart->CR1 |= (USART_CR1_TXEIE);
  // Enable UART IDLE interrupt
  uart->CR1 |= (USART_CR1_IDLEIE);

  // // Enable UART
  // uart->CR1 |= (USART_CR1_UE);
  return 0;
}

int hal_uart_fifo_push(hal_uart_fifo_t *self, uint8_t byte) {
  uint16_t next_head = (self->head + 1) % UART_BUFFER_SIZE;
  if (next_head != self->tail) {
    // FIFO is not full
    self->buffer[self->head] = byte;
    self->head = next_head;
    self->size++;
    return 0;
  }

  return 1;
}

int hal_uart_fifo_pop(hal_uart_fifo_t *self, uint8_t *byte) {
  if (self->size > 0) {
    // FIFO is not empty
    *byte = self->buffer[self->tail];
    self->tail = (self->tail + 1) % UART_BUFFER_SIZE;
    self->size--;
    return 0;
  }

  return 1;
}

uint16_t hal_uart_fifo_get_linear_size(hal_uart_fifo_t *self) {
  if (self->head >= self->tail) {
    return self->head - self->tail;
  } else {
    return UART_BUFFER_SIZE - self->tail;
  }
}

int hal_uart_write(hal_uart_t *self, uint8_t *data, uint16_t len) {
  uint16_t bytes_written = 0;

  for (uint16_t i = 0; i < len; i++) {
    hal_uart_fifo_push(&self->tx_fifo, data[i]);
    bytes_written++;
  }
  // Enable TXE interrupt
  // uart->CR1 |= USART_CR1_TXEIE;

  hal_uart_start_dma_tx_transfer(self);

  return bytes_written;
}

int hal_uart_read(hal_uart_t *self, uint8_t *data, uint16_t size) {
  uint16_t bytes_read = 0;

  while (bytes_read < size && self->rx_fifo.size > 0) {
    uint8_t byte;
    hal_uart_fifo_pop(&self->rx_fifo, &byte);
    data[bytes_read++] = byte;
  }

  // Return the number of bytes successfully read from the buffer
  return bytes_read;
}
