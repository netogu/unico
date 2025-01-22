#include "bsp.h"
#include "hal.h"
#include "rtos.h"
#include "shell.h"
#include "tasklist.h"

static TaskHandle_t task_cli_handle;
static StaticTask_t task_cli_tcb;
static StackType_t task_cli_stack[TASK_STACK_SIZE_CLI];
static void task_cli_app(void *parameters);

TaskHandle_t task_cli_init(void) {
  // Create the task

#if defined(SHELL_INTERFACE_USB)
  cli_usb_init();
  board_t *brd = board_get_handle();
  cli_uart_init(&brd->com.console);
#elif defined(SHELL_INTERFACE_UART)
  board_t *brd = board_get_handle();
  cli_uart_init(&brd->com.console);
#endif

  shell_init();

  task_cli_handle =
      xTaskCreateStatic(task_cli_app, xstr(TASK_NAME_CLI), TASK_STACK_SIZE_CLI,
                        NULL, TASK_PRIORITY_CLI, task_cli_stack, &task_cli_tcb);

  if (task_cli_handle != NULL) {
    return task_cli_handle;
  } else {
    return NULL;
  }
}

static void task_cli_app(void *parameters) {
  (void)parameters;

  vTaskDelay(100);
  // Print Shell Header
  for (size_t i = 0; i < strlen(shell_head); i++) {
#ifdef SHELL_INTERFACE_USB
    cli_usb_putc(shell_head[i]);
#else
    cli_uart_putc(shell_head[i]);
#endif
    vTaskDelay(1);
  }

  while (1) {
    shell_update();
    vTaskDelay(TASK_DELAY_CLI);
  }
}
