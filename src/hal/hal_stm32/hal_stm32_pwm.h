#ifndef __HAL_STM32_PWM_H
#define __HAL_STM32_PWM_H

#include "hal.h"

#define HRTIM_DT_COUNT_PER_NS(__DT_NS__)                                       \
  (__DT_NS__ / 0.73 + 0.5) // From RM0440 Table 221

enum pwm_hrtim_channels {
  PWM_HRTIM_TIM_A,
  PWM_HRTIM_TIM_B,
  PWM_HRTIM_TIM_C,
  PWM_HRTIM_TIM_D,
  PWM_HRTIM_TIM_E,
  PWM_HRTIM_TIM_F,
};

int hal_stm32_pwm_set_n_cycle_run(hal_pwm_t *self, uint32_t cycles);
int hal_stm32_pwm_enable_fault_input(hal_pwm_t *self, uint32_t fault);
int hal_stm32_pwm_swap_output(hal_pwm_t *self);
int hal_stm32_pwm_enable_adc_trigger(hal_pwm_t *self);
int hal_stm32_pwm_enable_period_interrupt(hal_pwm_t *self);

#endif // __HAL_STM32_PWM_H
