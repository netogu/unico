
//--------------------------------------------------------------------+
// HMI Task
//--------------------------------------------------------------------+

#include "bsp.h"
#include "hal.h"
#include "shell.h"
#include "tasklist.h"

static TaskHandle_t task_adcmon_handle;
static StaticTask_t task_adcmon_tcb;
static StackType_t task_adcmon_stack[TASK_STACK_SIZE_ADCMON];
static void task_adcmon_app(void *parameters);

TaskHandle_t task_adcmon_init(void) {
  // Create the task

  task_adcmon_handle = xTaskCreateStatic(
      task_adcmon_app, xstr(TASK_NAME_HMI), TASK_STACK_SIZE_HMI, NULL,
      TASK_PRIORITY_HMI, task_adcmon_stack, &task_adcmon_tcb);

  if (task_adcmon_handle != NULL) {
    return task_adcmon_handle;
  } else {
    return NULL;
  }
}

static void task_adcmon_app(void *parameters) {

  // board_t *brd = board_get_handle();

  /* Unused parameters. */
  (void)parameters;

  while (1) {

    vTaskDelay(TASK_DELAY_ADCMON);
  }
}
