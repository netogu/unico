#ifndef __TASKMSG_H
#define __TASKMSG_H

#include "lib/moc.h"
#include "rtos.h"
#include "shell.h"
#include <stdbool.h>

typedef struct pwmcon_msg_t {
  int32_t vq_mv;
  int32_t vd_mv;
  enum moc_foc_mode mode;
} pwmcon_msg_t;

// PWM Control Queues
extern QueueHandle_t task_pwmcon_queue;

void task_msg_init(void);
void task_pwmcon_msg_send(pwmcon_msg_t msg);
bool task_pwmcon_msg_receive(pwmcon_msg_t *msg);

#endif // __TASKMSG_H
