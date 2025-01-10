#ifndef __HAL_TIMER_H
#define __HAL_TIMER_H

#include "stm32g4.h"
#include <stdint.h>

typedef struct hal_timer_s {
  TIM_TypeDef *TIM;
  uint32_t period_us;
  enum {
    TIMER_STATE_UNINITIALIZED,
    TIMER_STATE_STOPPED,
    TIMER_STATE_RUNNING,
  } state;
  void (*on_timeout_cb)(void);
} hal_timer_t;

hal_timer_t *timer_create(uint32_t period_us, void (*on_timeout_cb)(void));
void timer_start(hal_timer_t *self);
void timer_stop(hal_timer_t *self);

#endif // __HAL_TIMER_H
