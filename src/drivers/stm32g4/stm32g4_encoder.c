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
  self->count_per_rev = count_per_rev;

  self->count = (uint32_t *)&TIM5->CNT;

  // Enable TIM5
  timer->CR1 |= TIM_CR1_CEN;

  return 0;
}

uint32_t encoder_read_count(encoder_t *self) { return *self->count; }

void encoder_load_count(encoder_t *self, uint32_t value) {
  *self->count = value;
}

int encoder_read_angle_q31(encoder_t *self, uint32_t *angle) {
  (void)self;
  (void)angle;
  return 0;
}
float encoder_read_angle_float(encoder_t *self) {
  return (float)*self->count / self->count_per_rev * 360.0;
}

uint32_t encoder_get_count_per_rev(encoder_t *self) {
  return self->_count_per_rev;
}
