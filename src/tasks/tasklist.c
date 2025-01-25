#include "tasklist.h"
#include "hal.h"

//--------------------------------------------------------------------+
// Task List
//--------------------------------------------------------------------+
const task_descriptor_t task_list[] = {
    {.name = xstr(TASK_NAME_CLI), .init = task_cli_init, .startup = true},
    {.name = xstr(TASK_NAME_USB),
     .init = task_usb_device_init,
     .startup = true},
    {.name = xstr(TASK_NAME_HMI), .init = task_hmi_init, .startup = true},
    {.name = xstr(TASK_NAME_PWM_CONTROL),
     .init = task_pwm_control_init,
     .startup = true}};

//--------------------------------------------------------------------+
// Bootup
//--------------------------------------------------------------------+

void bootup_system(void) {
  timer_us_init();
  printf(timestamp());
  printf("Booting System\r\n");
  task_manager_init();

  /* Start the scheduler. */
  vTaskStartScheduler();
}

const size_t task_list_size = sizeof(task_list) / sizeof(task_list[0]);
