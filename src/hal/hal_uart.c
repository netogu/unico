#include "bsp.h"
#include "hal.h"
#include "string.h"

#define NOCHAR '\0'

static StaticSemaphore_t uart_mutex_buffer;
SemaphoreHandle_t uart_mutex;

static hal_uart_t *serial_port = NULL;

int cli_uart_init(hal_uart_t *port) {

  uart_mutex = xSemaphoreCreateMutexStatic(&uart_mutex_buffer);
  if (uart_mutex == NULL) {
    // Error Creating UART Mutex
    while (1)
      ;
  }

  serial_port = port;

  return 0;
}

int cli_uart_putc(char tx_char) {

  int status = 0;
  if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
    status = hal_uart_write(serial_port, (uint8_t *)&tx_char, 1);
    xSemaphoreGive(uart_mutex);
  }
  return status;
}

int cli_uart_puts(const char *str) {
  int status = 0;
  if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
    size_t len = strlen(str);
    status = hal_uart_write(serial_port, (uint8_t *)str, len);
    xSemaphoreGive(uart_mutex);
  }
  return status;
}

#ifdef SHELL_INTERFACE_UART

int cli_printf(const char *format, ...) {

  int status = 0;
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
    status = uart_write(serial_port, (uint8_t *)buffer, strlen(buffer));
    xSemaphoreGive(uart_mutex);
  }
  return status;
}
#endif

char cli_uart_getc(void) {
  int status = 0;
  char readchar = NOCHAR;
  if (xSemaphoreTake(uart_mutex, 10) == pdTRUE) {
    status = hal_uart_read(serial_port, (uint8_t *)&readchar, 1);
    (void)status;
    xSemaphoreGive(uart_mutex);
  }
  return readchar;
}

uint32_t cli_uart_tx_pending(hal_uart_t *port) {
  // return serial_port->tx_fifo.size;
  uint32_t size = 0;

  if (port->tx_fifo.head >= serial_port->tx_fifo.tail) {
    size = port->tx_fifo.head - serial_port->tx_fifo.tail;
  } else {
    size = (UART_BUFFER_SIZE - port->tx_fifo.tail) + port->tx_fifo.head;
  }

  return size;
}
