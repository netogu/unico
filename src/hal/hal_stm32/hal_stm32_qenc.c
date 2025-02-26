// -------------------------------------------------+
// Encoder Implementation using 32bit TIM2 | TIM5
//
// -------------------------------------------------+

#include "hal_stm32_qenc.h"
#include "stm32g474xx.h"
#include <stddef.h>

int qenc_init(qenc_t *self) {
  if (self == NULL || self->timer == NULL) {
    return -1;
  }

  TIM_TypeDef *timer = (TIM_TypeDef *)self->timer;

  // Enable peripheral clock
  if (TIM2 == timer) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  } else if (TIM5 == timer) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
  }

  // Select TI1 and TI2 Source to TI1|2 as Inputs
  timer->CCMR1 = 0;
  timer->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  timer->CCMR1 |=
      0b1111 << TIM_CCMR1_IC1F_Pos | 0b1111 << TIM_CCMR1_CC2S_Pos; // Filtering

  // Set edge polarity in CCER - No filter
  //  CC1P & CC1NP = 0  : Non-Inverted Rising-Edge
  //  CC2P & CC2NP = 0  : Non-Inverted Rising-Edge
  timer->CCER &=
      ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);

  // Initialize TIM - Slave mode selection
  timer->SMCR &= ~TIM_SMCR_SMS_Msk;
  timer->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
  // timer->SMCR |=
  //     TIM_SMCR_ETF_3 | TIM_SMCR_ETF_2 | TIM_SMCR_ETF_1 | TIM_SMCR_ETF_0;
  // timer->SMCR |= 0b1111 << TIM_SMCR_SMS_Pos;

  timer->PSC = 0;
  timer->CCER |=
      TIM_CCER_CC1E | TIM_CCER_CC2E; // Enable capture on both channels
  // Limit Range to 0

  // timer->ARR = self->cpr;
  timer->ARR = 0xFFFFFFFF;

  timer->CNT = 0;

  // Enable TIM5
  timer->CR1 |= TIM_CR1_CEN;

  return 0;
}

void qenc_run(qenc_t *self) {
  TIM_TypeDef *timer = (TIM_TypeDef *)self->timer;
  timer->CR1 |= TIM_CR1_CEN;
}

void qenc_stop(qenc_t *self) {
  TIM_TypeDef *timer = (TIM_TypeDef *)self->timer;
  timer->CR1 &= ~TIM_CR1_CEN;
}

void qenc_reset(qenc_t *self) {
  TIM_TypeDef *timer = (TIM_TypeDef *)self->timer;
  timer->CNT = 0;
}
void qenc_load(qenc_t *self, uint32_t count) {
  TIM_TypeDef *timer = (TIM_TypeDef *)self->timer;
  timer->CNT = count;
}
int32_t qenc_read(qenc_t *self) {
  if (self == NULL || self->timer == NULL) {
    return 0; // Error
  }

  TIM_TypeDef *timer = (TIM_TypeDef *)self->timer;
  return timer->CNT;
}

int qenc_timer_set_frequency(qenc_t *self, uint32_t hz);

// static uint32_t encoder_read(void) { return TIM5->CNT; }
//
// static void encoder_load(uint32_t value) { TIM5->CNT = value; }
