#include "encoder.h"
#include "stm32g4.h"
#include <stdint.h>

// Encoder Implementation using 32bit TIM5
int encoder_init(encoder_t *self, uint32_t count_per_rev) {

  TIM_TypeDef *timer = TIM5;

  // Enable peripheral clock
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;

  // Select TI1 and TI2 Source to TI1|2 as Inputs
  timer->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;

  // Set edge polarity in CCER - No filter
  //  CC1P & CC1NP = 0  : Non-Inverted Rising-Edge
  //  CC2P & CC2NP = 0  : Non-Inverted Rising-Edge
  timer->CCER &=
      ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);

  // Initialize TIM5 - Slave mode selection
  timer->SMCR |= TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0;
  // timer->SMCR |= 0b1111 << TIM_SMCR_SMS_Pos;

  // Limit Range to count_per_rev
  timer->ARR = count_per_rev;
  self->cpr = count_per_rev;

  // Enable TIM5
  timer->CR1 |= TIM_CR1_CEN;

  return 0;
}

void encoder_update(encoder_t *self) { self->count = TIM5->CNT; }

void encoder_load_count(encoder_t *self, uint32_t value) {
  self->count = value;
  TIM5->CNT = value;
}
