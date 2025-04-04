#include "hal_stm32_pwr.h"

void pwr_set_vos_range(enum pwr_vos_ranges scale) {
  Modify_register_field(PWR->CR1, PWR_CR1_VOS, scale);
  // Wait for regulaor to stabilize
  while (PWR->SR1 & PWR_SR2_VOSF)
    ;
}

void pwr_enable_boost(void) { PWR->CR5 &= ~PWR_CR5_R1MODE; }

void pwr_disable_boost(void) { PWR->CR5 |= PWR_CR5_R1MODE; }
