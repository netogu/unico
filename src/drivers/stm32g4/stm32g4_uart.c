/******************************************************************************
 * File: uart.c
 * Description: Implementation of UART driver for STM32G4 series of
 *microcontrollers.
 *
 * Author: Ernesto Gonzalez Urdaneta
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "stm32g4_uart.h"
#include "stm32g4_common.h"

#define UART_CLOCK_SOURCE_PCLK 0
#define UART_INPUT_CLOCK_PRESCALER_NONE 0

#define USART3_DMA_RX_REQ_NUM 28
#define USART3_DMA_TX_REQ_NUM 29
#define LPUART1_DMA_RX_REQ_NUM 34
#define LPUART1_DMA_TX_REQ_NUM 35

static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

static void lpuart_init(uart_t *self) {
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

static void usart3_init(uart_t *self) {
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

static void usart_enable_dma(uart_t *self) {

  USART_TypeDef *uart_base = (USART_TypeDef *)self->uart_base;

  // Turn on DMA1 and DMAMUX1 clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
  uint32_t tmpreg = RCC->AHB1ENR;
  (void)tmpreg;

  // Configure DMA1 Channel 2 for UART RX
  DMA1_Channel2->CPAR = (uint32_t)&(uart_base->RDR);
  DMA1_Channel2->CMAR = (uint32_t)self->rx_fifo.buffer_ptr;
  DMA1_Channel2->CNDTR = UART_RX_BUFFER_SIZE;
  DMA1_Channel2->CCR = 0;
  // Circular mode, enable transfer complete and half transfer interrupts
  DMA1_Channel2->CCR |=
      DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_HTIE;
  // Route USART3 RX DMA REQ to DMA1 Channel 2

  // Configure DMA1 Channel 3 for UART TX
  DMA1_Channel3->CPAR = (uint32_t)&(uart_base->TDR);
  DMA1_Channel3->CMAR = 0;
  DMA1_Channel3->CNDTR = 0;
  DMA1_Channel3->CCR = 0;
  // Enable memory increment and set direction to memory-to-peripheral
  DMA1_Channel3->CCR |=
      DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_HTIE;

  // Route DMA Requests
  // DMAMUX1_Channel1->CCR = USART3_DMA_RX_REQ_NUM;
  DMAMUX1_Channel1->CCR = LPUART1_DMA_RX_REQ_NUM;
  // DMAMUX1_Channel2->CCR = USART3_DMA_TX_REQ_NUM;
  DMAMUX1_Channel2->CCR = LPUART1_DMA_TX_REQ_NUM;

  // Enable DMA1 Channel 2
  DMA1_Channel2->CCR |= DMA_CCR_EN;
  // Enable DMA1 Channel 3
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  // Enable USART DMA
  uart_base->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);

  // Enable interrupts
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

int uart_dma_transfer(uart_t *self) {

  // TODO: check combined busy flags for DMA and UART instead
  if (DMA1_Channel3->CNDTR) {
    // DMA is still busy
    return 1;
  }

  ul_fifo_t *fifo = &self->tx_fifo;

  uint8_t *buffer_head_ptr =
      (uint8_t *)fifo->buffer_ptr + (fifo->head * fifo->element_size);

  size_t bytes_to_transfer = 0;

  if (fifo->tail > fifo->head) {
    // Contiguous data: head to tail
    bytes_to_transfer = (fifo->tail - fifo->head) * fifo->element_size;
  } else {
    // Wrap-around data: head to end of buffer
    bytes_to_transfer = (fifo->capacity - fifo->head) * fifo->element_size;
  }

  if (bytes_to_transfer > 0) {
    // Some data to transfer

    // Disable Interrupts
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Disable the DMA channel
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;

    // Clear interrupt flags
    DMA1->IFCR =
        DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3 | DMA_IFCR_CGIF3;

    // Set the memory address to buffer head
    DMA1_Channel3->CMAR = (uint32_t)buffer_head_ptr;
    // Set the number of data items to transfer
    DMA1_Channel3->CNDTR = bytes_to_transfer;
    self->dma_tx_pending = bytes_to_transfer;
    // Enable the DMA channel
    DMA1_Channel3->CCR |= DMA_CCR_EN;
    // Re-enable interrupts
    __set_PRIMASK(primask);
  }

  return 0;
}

int uart_fifo_update_on_dma(uart_t *self) {
  ul_fifo_t *fifo = &self->tx_fifo;
  size_t dma_tx_pending_update = DMA1_Channel3->CNDTR;
  size_t transfered_bytes =
      (self->dma_tx_pending >= dma_tx_pending_update)
          ? (self->dma_tx_pending - dma_tx_pending_update)
          : (self->dma_tx_pending + (fifo->capacity - dma_tx_pending_update));
  // Update tail pointer
  fifo->tail = (fifo->tail + transfered_bytes) % fifo->capacity;
  // Update count
  fifo->count = (fifo->count + transfered_bytes) > fifo->capacity
                    ? fifo->capacity
                    : fifo->count + transfered_bytes;

  self->dma_tx_pending -= transfered_bytes;

  return 0;
}

void uart_dma_receive(uart_t *self) {
  ul_fifo_t *fifo = &self->rx_fifo;
  fifo->tail = fifo->capacity - DMA1_Channel2->CNDTR;
  fifo->count = (fifo->tail >= fifo->head)
                    ? (fifo->tail - fifo->head)
                    : (fifo->capacity - fifo->head + fifo->tail);
  if (fifo->count > fifo->capacity) {
    fifo->count = fifo->capacity;
  }
}

void uart_init_dma(uart_t *self, uart_config_t config) {
  USART_TypeDef *uart_base = (USART_TypeDef *)self->uart_base;

  self->dma_tx_pending = 0;

  uart_init(self, config);
  usart_enable_dma(self);

  // Enable UART
  uart_base->CR1 |= (USART_CR1_UE);
}

void uart_init(uart_t *self, uart_config_t config) {

  USART_TypeDef *uart_base = (USART_TypeDef *)self->uart_base;

  ul_fifo_init(&self->tx_fifo, uart_tx_buffer,
               sizeof(uart_tx_buffer) / sizeof(uart_tx_buffer[0]),
               sizeof(uart_tx_buffer[0]));
  ul_fifo_init(&self->rx_fifo, uart_rx_buffer,
               sizeof(uart_rx_buffer) / sizeof(uart_rx_buffer[0]),
               sizeof(uart_rx_buffer[0]));

  if (uart_base == LPUART1) {
    lpuart_init(self);
  } else if (uart_base == USART3) {
    usart3_init(self);
  }

  // Set UART Prescaler
  uart_base->PRESC = 0x00;
  uart_base->PRESC |= (UART_INPUT_CLOCK_PRESCALER_NONE & 0x0F);

  // Disable UART
  uart_base->CR1 &= ~(USART_CR1_UE);
  uart_base->CR1 = 0x00;

  if (uart_base == LPUART1) {
    // Set baudrate LPUART Specific
    uint32_t usartdiv = SystemCoreClock / config.baudrate * 256;
    uart_base->BRR = usartdiv & 0x0FFFFF; // 20 bits
  } else {
    // Set baudrate
    uint32_t usartdiv = SystemCoreClock / config.baudrate;
    uart_base->BRR = usartdiv & 0x0FFFF; // 16 bits
  }

  // Set data bits
  // 8 bits by default
  uart_base->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);
  if (config.word_length == UART_DATA_BITS_9) {
    uart_base->CR1 |= (USART_CR1_M0);
  } else if (config.word_length == UART_DATA_BITS_7) {
    uart_base->CR1 |= (USART_CR1_M1);
  }

  // Set parity
  uart_base->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
  if (config.parity == UART_PARITY_EVEN) {
    uart_base->CR1 |= (USART_CR1_PCE);
    uart_base->CR1 |= (config.parity << USART_CR1_PS_Pos);
  } // else LPUART_PARITY_NONE

  // Set stop bits
  uart_base->CR2 &= ~(USART_CR2_STOP);
  uart_base->CR2 |= (config.stop_bits << USART_CR2_STOP_Pos);

  // Set Flow Control
  uart_base->CR3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);
  if (config.flow_control & UART_FLOW_CONTROL_RTS) {
    uart_base->CR3 |= (USART_CR3_RTSE);
  } else if (config.flow_control & UART_FLOW_CONTROL_CTS) {
    uart_base->CR3 |= (USART_CR3_CTSE);
  } else if (config.flow_control & UART_FLOW_CONTROL_RTS_CTS) {
    uart_base->CR3 |= (USART_CR3_RTSE | USART_CR3_CTSE);
  } // else LPUART_FLOW_CONTROL_NONE

  // Set mode
  uart_base->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);
  if (config.mode == UART_MODE_RX) {
    uart_base->CR1 |= (USART_CR1_RE);
  } else if (config.mode == UART_MODE_TX) {
    uart_base->CR1 |= (USART_CR1_TE);
  } else if (config.mode == UART_MODE_RX_TX) {
    uart_base->CR1 |= (USART_CR1_RE | USART_CR1_TE);
  } // else LPUART_MODE_NONE

#if UART_RXNEIE_ON
  // Enable UART RXNE interrupt
  uart_base->CR1 |= (USART_CR1_RXNEIE);
#endif
#if UART_TXEIE_ON
  // Enable UART TXE interrupt
  uart_base->CR1 |= (USART_CR1_TXEIE);
#endif
  // Enable UART IDLE interrupt
  uart_base->CR1 |= (USART_CR1_IDLEIE);

  // // Enable UART
  // uart_base->CR1 |= (USART_CR1_UE);
}

size_t uart_write(uart_t *self, uint8_t *data, size_t size) {
  size_t bytes_written = 0;

  for (size_t i = 0; i < size; i++) {
    ul_fifo_enqueue(&self->tx_fifo, &data[i]);
    bytes_written++;
  }
  // Enable TXE interrupt
  // uart_base->CR1 |= USART_CR1_TXEIE;

  uart_dma_transfer(self);

  return bytes_written;
}

size_t uart_read(uart_t *self, uint8_t *data, size_t size) {
  size_t bytes_read = 0;

  while (bytes_read < size && self->rx_fifo.count > 0) {
    ul_fifo_dequeue(&self->rx_fifo, &data);
    bytes_read++;
  }

  // Return the number of bytes successfully read from the buffer
  return bytes_read;
}
