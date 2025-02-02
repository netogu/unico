#ifndef TASK_LIST_H
#define TASK_LIST_H

#include "rtos.h"
#include "shell.h"
#include <stdbool.h>

//--------------------------------------------------------------------+
// Task List
//--------------------------------------------------------------------+

#define TASK_NAME_TASK_MANAGER taskman
#define TASK_NAME_USB usb
#define TASK_NAME_CLI cli
#define TASK_NAME_HMI hmi
#define TASK_NAME_PWM_CONTROL pwmcon

#define TASK_PRIORITY_TASK_MANAGER 1
#define TASK_PRIORITY_USB 3
#define TASK_PRIORITY_CLI 4
#define TASK_PRIORITY_HMI 1
#define TASK_PRIORITY_PWM_CONTROL 3

#define TASK_REPEAT_TASK_MANAGER 1
#define TASK_REPEAT_USB 2
#define TASK_REPEAT_CLI 5
#define TASK_REPEAT_HMI 1
#define TASK_REPEAT_PWM_CONTROL 1

#define TASK_DELAY_TASK_MANAGER 5
#define TASK_DELAY_USB 1
#define TASK_DELAY_CLI 4
#define TASK_DELAY_HMI 1000
#define TASK_DELAY_PWM_CONTROL 10

#define TASK_STACK_SIZE_TASK_MANAGER 1024
#define TASK_STACK_SIZE_USB 1024
#define TASK_STACK_SIZE_CLI 1024
#define TASK_STACK_SIZE_HMI 1024
#define TASK_STACK_SIZE_PWM_CONTROL 1024

//--------------------------------------------------------------------+
// Task Functions
//--------------------------------------------------------------------+

TaskHandle_t task_manager_init(void);
TaskHandle_t task_usb_device_init(void);
TaskHandle_t task_cli_init(void);
TaskHandle_t task_hmi_init(void);
TaskHandle_t task_pwm_control_init(void);
TaskHandle_t task_coms_manager_init(void);

// TODO: Write a task that manages communications

//--------------------------------------------------------------------+
// Task Descriptors
//--------------------------------------------------------------------+

// Task Function pointer typedef
typedef TaskHandle_t (*task_init_t)(void);

typedef struct task_descriptor_s {
  const char *const name;
  const bool startup;
  task_init_t init;
} task_descriptor_t;

extern const task_descriptor_t task_list[];
extern const size_t task_list_size;

void bootup_system(void);

#endif // TASK_LIST_H
