
//--------------------------------------------------------------------+
// HMI Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "shell.h"
#include "tasklist.h"

static TaskHandle_t task_hmi_handle;
static StaticTask_t task_hmi_tcb;
static StackType_t task_hmi_stack[TASK_STACK_SIZE_HMI];
static void task_hmi_app(void *parameters);

TaskHandle_t task_hmi_init(void) {
  // Create the task

  task_hmi_handle =
      xTaskCreateStatic(task_hmi_app, xstr(TASK_NAME_HMI), TASK_STACK_SIZE_HMI,
                        NULL, TASK_PRIORITY_HMI, task_hmi_stack, &task_hmi_tcb);

  if (task_hmi_handle != NULL) {
    return task_hmi_handle;
  } else {
    return NULL;
  }
}

static void task_hmi_app(void *parameters) {

  board_t *brd = board_get_handle();

  /* Unused parameters. */
  (void)parameters;

  while (1) {
    gpio_pin_toggle(&brd->dio.led_green);
    // gpio_pin_toggle(&brd->io.led_red);
    // gpio_pin_toggle(&brd->io.led_blue);
    gpio_pin_set(&brd->dio.led_blue);
    gpio_pin_set(&brd->dio.led_red);

    vTaskDelay(TASK_DELAY_HMI);
  }
}
