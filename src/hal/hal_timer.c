#include "hal.h"
#include "stm32g474xx.h"

#define HAL_TIMER_MAX_INSTANCES 2
#define HAL_TIMER_INTERRUPT_PRIORITY 1

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

static union {
  struct {
    uint32_t low_u32;
    uint32_t high_u32;
  };
  uint64_t cnt;
} timer_us_ticks;

static hal_timer_t timer_list[HAL_TIMER_MAX_INSTANCES] = {
    {
        .TIM = TIM6,
        .period_us = 0,
        .state = TIMER_STATE_UNINITIALIZED,
        .on_timeout_cb = NULL,
    },
    {
        .TIM = TIM7,
        .period_us = 0,
        .state = TIMER_STATE_UNINITIALIZED,
        .on_timeout_cb = NULL,
    }};

static int _timer_stm32g4_init(hal_timer_t *self);

hal_timer_t *timer_create(uint32_t period_us, void (*on_timeout_cb)(void)) {
  // Create a new timer

  hal_timer_t *timer_handle = NULL;

  // Check if there is a free timer
  for (uint32_t i = 0; i < HAL_TIMER_MAX_INSTANCES; i++) {
    if (timer_list[i].on_timeout_cb == NULL) {
      // Found a free timer
      timer_list[i].period_us = period_us;
      timer_list[i].on_timeout_cb = on_timeout_cb;
      timer_list[i].state = TIMER_STATE_UNINITIALIZED;

      _timer_stm32g4_init(&timer_list[i]);

      timer_handle = &timer_list[i];

    } else {
      // No free timers
    }
  }

  return timer_handle;
}

static int _timer_stm32g4_init(hal_timer_t *self) {

  // Initialize the timer
  // Set the timer to the stopped state
  self->state = TIMER_STATE_STOPPED;

  if (self->TIM == TIM6) {
    // Enable the timer clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    NVIC_SetPriority(TIM6_DAC_IRQn, HAL_TIMER_INTERRUPT_PRIORITY);
  } else if (self->TIM == TIM7) {
    // Enable the timer clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
    NVIC_EnableIRQ(TIM7_DAC_IRQn);
    NVIC_SetPriority(TIM7_DAC_IRQn, HAL_TIMER_INTERRUPT_PRIORITY);
  } else {
    // Invalid timer
    return -1;
  }

  // Set the prescaler to 1MHz
  self->TIM->PSC = 170 - 1;
  // Set the period
  self->TIM->ARR = self->period_us;

  // Enable the timer interrupt
  self->TIM->DIER |= TIM_DIER_UIE;

  return 0;
}

void timer_start(hal_timer_t *self) {
  // Start the timer
  self->state = TIMER_STATE_RUNNING;
  self->TIM->CR1 |= TIM_CR1_CEN;
}

void timer_stop(hal_timer_t *self) {
  // Stop the timer
  self->state = TIMER_STATE_STOPPED;
  self->TIM->CR1 &= ~TIM_CR1_CEN;
}

void timer_us_init(void) {
  // Initialize the timer

  timer_us_ticks.cnt = 0;

  // Configuring TIM as a 32-bit microsecond timer

  // Enable TIM clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

  // Set the prescaler to 1MHz
  TIM5->PSC = 170 - 1;
  // Set Period to max value
  TIM5->ARR = 0xFFFFFFFF;

  // Enable Periodic Interrupt
  TIM5->DIER |= TIM_DIER_UIE;
  NVIC_EnableIRQ(TIM5_IRQn);

  // Enable the timer
  TIM5->CR1 |= TIM_CR1_CEN;
}

uint64_t timer_us_get(void) {
  // Get the TIM5 value
  timer_us_ticks.low_u32 = TIM5->CNT;
  return timer_us_ticks.cnt;
}

// ISR for TIM5
void TIM5_IRQHandler(void) {
  // Clear the interrupt
  TIM5->SR &= ~TIM_SR_UIF;
  timer_us_ticks.high_u32++;
}

// ISR for TIM6
void TIM6_DAC_IRQHandler(void) {
  // Clear the interrupt
  TIM6->SR &= ~TIM_SR_UIF;

  // Call the callback
  timer_list[0].on_timeout_cb();
}

// ISR for TIM7
void TIM7_DAC_IRQHandler(void) {
  // Clear the interrupt
  TIM7->SR &= ~TIM_SR_UIF;

  // Call the callback
  timer_list[1].on_timeout_cb();
}
