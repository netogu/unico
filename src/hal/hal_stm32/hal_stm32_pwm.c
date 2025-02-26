
#include "hal_stm32_pwm.h"
#include "hal.h"

static void _hrtim1_init(void) {

  // Enable HRTIM clock source (RCC)
  // - check that fHRTIM won't exceed range of DLL lock
  RCC->APB2ENR |= RCC_APB2ENR_HRTIM1EN;

  // Start DLL calibration by setting CAL in HRTIM_DLLCR
  HRTIM1->sCommonRegs.DLLCR |= HRTIM_DLLCR_CAL;

  // Wait for HR unit is ready by waiting for DLLRDY flag,
  // can keep doing things but must be ready before starting timers
  while (!(HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY)) {
    // Optional timeout mechanism for robustness
  }
}

static int _pwm_enable_outputs(hal_pwm_t *self) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {
    // Enable outputs
    HRTIM1->sCommonRegs.OENR |= 1 << (HRTIM_OENR_TA1OEN_Pos + 2 * pwm_channel);
    HRTIM1->sCommonRegs.OENR |= 1 << (HRTIM_OENR_TA2OEN_Pos + 2 * pwm_channel);
  }

  return 0;
}

int hal_pwm_set_frequency(hal_pwm_t *self, uint32_t freq_hz) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {
    uint32_t prescale =
        HRTIM1->sTimerxRegs[pwm_channel].TIMxCR & HRTIM_TIMCR_CK_PSC;
    HRTIM1->sTimerxRegs[pwm_channel].PERxR =
        SystemCoreClock / freq_hz * 32 >> (prescale + 1);
  }
  return 0;
}

/**
 * @brief Set PWM duty cycle
 *
 * @param self      pwm_t object
 * @param duty_u    Duty cycle normalized to 1.0
 * @return int      0 on success
 */

int hal_pwm_set_duty_f32(hal_pwm_t *self, float duty_u) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {
    const float duty_max = 0.98;
    const float duty_min = 0.02;

    if (duty_u < duty_min) {
      duty_u = duty_min;
    } else if (duty_u > duty_max) {
      duty_u = duty_max;
    }
    // } else {
    //   // Invalid duty cycle
    //   return -1;
    // }

    // read period
    uint32_t period_reg = HRTIM1->sTimerxRegs[pwm_channel].PERxR;
    float duty_period = (period_reg) * (duty_u);

    // Center edge modulation by default
    // uint32_t cmp = period_reg/2;
    uint32_t cmp = (uint32_t)(duty_period + 0.5f);
    HRTIM1->sTimerxRegs[pwm_channel].CMP1xR = cmp;
  }

  return 0;
}

int hal_pwm_swap_output(hal_pwm_t *self) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {
    // Swap PWM outputs
    HRTIM1->sCommonRegs.CR2 |= (1 << (HRTIM_CR2_SWPA_Pos + pwm_channel));
    // Update registers
    HRTIM1->sCommonRegs.CR2 |= (1 << (HRTIM_CR2_TASWU_Pos + pwm_channel));
  }
  return 0;
}

int hal_pwm_enable_fault_input(hal_pwm_t *self, uint32_t fault) {
  (void)fault;

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];

    // Set Input Source to FLT pin
    HRTIM1->sCommonRegs.FLTINR2 &=
        ~(HRTIM_FLTINR2_FLT5SRC_0_Msk | HRTIM_FLTINR2_FLT5SRC_1_Msk);

    // Set Input Polarity
    HRTIM1->sCommonRegs.FLTINR2 &= ~HRTIM_FLTINR2_FLT5P_Msk; // Active Low

    // Configure input filter
    HRTIM1->sCommonRegs.FLTINR2 |=
        2 << HRTIM_FLTINR2_FLTSD_Pos; // fFLTS = fHRTIM/4
    HRTIM1->sCommonRegs.FLTINR2 &= ~HRTIM_FLTINR2_FLT5F_Msk;
    HRTIM1->sCommonRegs.FLTINR2 |=
        6 << HRTIM_FLTINR2_FLT5F_Pos; // fsampling = fFLTS/4 * 6

    // Configure Blanking sources
    // -- None --

    // Engage fault input
    HRTIM1->sCommonRegs.FLTINR2 |= HRTIM_FLTINR2_FLT5E;

    // Configure PWM faulted states
    tim_regs->OUTxR &= ~HRTIM_OUTR_FAULT1_Msk;
    tim_regs->OUTxR &= ~HRTIM_OUTR_FAULT2_Msk;
    tim_regs->OUTxR |= 0b10 << HRTIM_OUTR_FAULT1_Pos; // Inactive fault state
    tim_regs->OUTxR |= 0b10 << HRTIM_OUTR_FAULT2_Pos; // Inactive fault state

    // Enable fault
    tim_regs->FLTxR |= HRTIM_FLTR_FLT5EN;
  }

  return 0;
}

int hal_pwm_init(hal_pwm_t *self, uint32_t freq_hz, uint32_t dt_ns) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {

    _hrtim1_init();

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];

    // Configure PWM Mode

    uint32_t period = 0;
    uint32_t prescale = 0;

    if (freq_hz < 83000) {
      prescale = 1;
    }
    if (freq_hz < 41500) {
      prescale = 2;
    }
    if (freq_hz < 20800) {
      prescale = 3;
    }
    if (freq_hz < 10400) {
      prescale = 4;
    }
    if (freq_hz < 5190) {
      prescale = 5;
    }
    if (freq_hz < 2590) {
      prescale = 6;
    }
    if (freq_hz < 1300) {
      prescale = 7;
    }

    tim_regs->TIMxCR &= ~HRTIM_TIMCR_CK_PSC;
    tim_regs->TIMxCR |= prescale << HRTIM_TIMCR_CK_PSC_Pos;

    // Set PWM Mode to Center Aligned
    tim_regs->TIMxCR2 |= HRTIM_TIMCR2_UDM;
    // tim_regs->SETx1R = HRTIM_SET1R_CMP1;
    tim_regs->RSTx1R = HRTIM_RST1R_CMP1;
    period = (SystemCoreClock / freq_hz * (32 >> prescale) + 1) / 2;

    tim_regs->PERxR = period;
    tim_regs->CMP1xR = 0;

    // Pre-load enable update on reset/roll-over, continuous mode
    tim_regs->TIMxCR |=
        (HRTIM_TIMCR_PREEN | HRTIM_TIMCR_TRSTU | HRTIM_TIMCR_CONT);
    // Output roll-over mode
    tim_regs->TIMxCR2 &=
        ~(HRTIM_TIMCR2_ADROM | HRTIM_TIMCR2_OUTROM | HRTIM_TIMCR2_ROM);
    tim_regs->TIMxCR2 |= (0b10 << HRTIM_TIMCR2_ADROM_Pos);  // ADC Sample @PER
    tim_regs->TIMxCR2 |= (0b10 << HRTIM_TIMCR2_OUTROM_Pos); // @PER
    tim_regs->TIMxCR2 |= (0b10 << HRTIM_TIMCR2_ROM_Pos);    // @PER

    // Configure Timer outputs, polarity, then FAULT and IDLE states

    // Configure PWM Output : Reset on match, Set on Period

    // tim_regs->RSTx2R = HRTIM_RST1R_CMP1;
    // tim_regs->SETx2R = HRTIM_SET1R_CMP1;

    // Configure Deadtime
    tim_regs->DTxR |= (1 << HRTIM_DTR_DTPRSC_Pos); // tDTG = tHRTIM/4
    // From Rm0440 table 221
    uint32_t dt_cnt = (uint32_t)(dt_ns / 1.47 + 0.5);

    if (dt_cnt > 255)
      return -1;

    tim_regs->DTxR |=
        (dt_cnt << HRTIM_DTR_DTF_Pos) | (dt_cnt << HRTIM_DTR_DTR_Pos);
    // Enable Deadtime
    tim_regs->OUTxR |= HRTIM_OUTR_DTEN;

    // Configure PWM Polarity
  }

  return 0;
}

int hal_pwm_start(hal_pwm_t *self) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {
    // Enable Outputs
    _pwm_enable_outputs(self);
    // Start Timer
    HRTIM1->sMasterRegs.MCR |= 1 << (HRTIM_MCR_TACEN_Pos + pwm_channel);
  }

  return 0;
}

int hal_pwm_stop(hal_pwm_t *self) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {
    HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN + pwm_channel);
  }

  return 0;
}

int hal_stm32_pwm_enable_period_interrupt(hal_pwm_t *self) {

  uint16_t pwm_channel = self->channel;
  HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];
  tim_regs->TIMxDIER |= HRTIM_TIMDIER_RSTIE;

  switch (pwm_channel) {
  case PWM_HRTIM_TIM_A:
    NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
    break;
  case PWM_HRTIM_TIM_B:
    NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
    break;
  case PWM_HRTIM_TIM_C:
    NVIC_EnableIRQ(HRTIM1_TIMC_IRQn);
    break;
  case PWM_HRTIM_TIM_D:
    NVIC_EnableIRQ(HRTIM1_TIMD_IRQn);
    break;
  case PWM_HRTIM_TIM_E:
    NVIC_EnableIRQ(HRTIM1_TIME_IRQn);
    break;
  case PWM_HRTIM_TIM_F:
    NVIC_EnableIRQ(HRTIM1_TIMF_IRQn);
    break;
  default:
    break;
  }

  return 0;
}

int hal_pwm_set_n_cycle_run(hal_pwm_t *self, uint32_t cycles) {

  uint16_t pwm_channel = self->channel;

  if (self->regs == HRTIM1) {

    HRTIM_Timerx_TypeDef *tim_regs = &HRTIM1->sTimerxRegs[pwm_channel];

    // Reset Timer
    hal_pwm_stop(self);

    // Set PWM to Continuous mode
    tim_regs->TIMxCR |= HRTIM_TIMCR_CONT;
    // Roll-over mode counter = zero
    tim_regs->TIMxCR2 |= 1 << HRTIM_TIMCR2_ROM_Pos;
    // Set Repetition counter
    tim_regs->REPxR = cycles - 2;
    // Force Update
    HRTIM1->sCommonRegs.CR2 |= HRTIM_CR2_TASWU + pwm_channel;

    // Enable REP Interrupt
    tim_regs->TIMxDIER |= HRTIM_TIMDIER_REPIE;

    switch (pwm_channel) {
    case PWM_HRTIM_TIM_A:
      NVIC_EnableIRQ(HRTIM1_TIMA_IRQn);
      break;
    case PWM_HRTIM_TIM_B:
      NVIC_EnableIRQ(HRTIM1_TIMB_IRQn);
      break;
    case PWM_HRTIM_TIM_C:
      NVIC_EnableIRQ(HRTIM1_TIMC_IRQn);
      break;
    case PWM_HRTIM_TIM_D:
      NVIC_EnableIRQ(HRTIM1_TIMD_IRQn);
      break;
    case PWM_HRTIM_TIM_E:
      NVIC_EnableIRQ(HRTIM1_TIME_IRQn);
      break;
    case PWM_HRTIM_TIM_F:
      NVIC_EnableIRQ(HRTIM1_TIMF_IRQn);
      break;
    default:
      break;
    }
  }

  return 0;
}

int hal_stm32_pwm_enable_adc_trigger(hal_pwm_t *self) {

  if (self->regs != HRTIM1) {
    return -1;
  }

  uint16_t pwm_channel = self->channel;

  switch (pwm_channel) {
  case PWM_HRTIM_TIM_A:
    HRTIM1->sCommonRegs.CR1 &= ~(HRTIM_CR1_ADC1USRC_Msk);
    HRTIM1->sCommonRegs.CR1 |= (1) << HRTIM_CR1_ADC1USRC_Pos;
    HRTIM1->sCommonRegs.ADC2R |= HRTIM_ADC2R_AD2TAPER;
    // HRTIM1->sCommonRegs.ADC1R |= HRTIM_ADC1R_AD1TARST;
    break;
  case PWM_HRTIM_TIM_B:
    // HRTIM1->sCommonRegs.CR1 = (1) << HRTIM_CR1_ADC1USRC_Pos;
    // HRTIM1->sCommonRegs.ADC1R |= HRTIM_ADC1R_AD1TBRST;
    break;
  case PWM_HRTIM_TIM_F:
    // HRTIM1->sCommonRegs.CR1 = (1) << HRTIM_CR1_ADC1USRC_Pos;
    // HRTIM1->sCommonRegs.ADC1R |= HRTIM_ADC1R_AD1TFRST;
    break;
  default:
    return -1;
    break;
  }

  return 0;
}

int hal_pwm_3ph_init(hal_pwm_3ph_t *self, uint32_t freq_hz, uint32_t dt_ns) {

  for (int i = 0; i < 3; i++) {
    hal_pwm_init(&self->pwm[i], freq_hz, dt_ns);
  }

  return 0;
}

int hal_pwm_3ph_start(hal_pwm_3ph_t *self) {

  uint32_t mcr_reg = HRTIM1->sMasterRegs.MCR;

  for (int i = 0; i < 3; i++) {
    _pwm_enable_outputs(&self->pwm[i]);
    mcr_reg |= 1 << (HRTIM_MCR_TACEN_Pos + self->pwm[i].channel);
  }

  // Start Timer
  HRTIM1->sMasterRegs.MCR |= mcr_reg;

  return 0;
}

int hal_pwm_3ph_stop(hal_pwm_3ph_t *self) {

  uint32_t mcr_reg = HRTIM1->sMasterRegs.MCR;

  for (int i = 0; i < 3; i++) {
    hal_pwm_stop(&self->pwm[i]);
    mcr_reg &= ~(HRTIM_MCR_TACEN + self->pwm[i].channel);
  }

  HRTIM1->sMasterRegs.MCR = mcr_reg;

  return 0;
}

int hal_pwm_3ph_set_frequency(hal_pwm_3ph_t *self, uint32_t freq_hz) {

  for (int i = 0; i < 3; i++) {
    hal_pwm_set_frequency(&self->pwm[i], freq_hz);
  }

  return 0;
}

int hal_pwm_3ph_set_duty_f32(hal_pwm_3ph_t *self, float duty_abc[]) {

  (void)self;

  for (int i = 0; i < 3; i++) {
    hal_pwm_set_duty_f32(&self->pwm[i], duty_abc[i]);
  }

  return 0;
}

__attribute__((unused)) static void pwm_dac_init(void) {
  // Enable TIM20 APB Clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM20EN;
  // Enable Auto-Reload
  TIM20->CR1 |= TIM_CR1_ARPE;
  // Set count mode to up-count
  TIM20->CR1 &= ~TIM_CR1_DIR;
  // Set Prescaler
  TIM20->PSC = 0;
  // Set Period
  TIM20->ARR = SystemCoreClock / 50000;
  // Set Duty Cycle to 25%
  TIM20->CCR3 = TIM20->ARR >> 2;
  // Set CH3 output mode to PWM
  TIM20->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
  // Preload disable
  TIM20->CCMR2 &= ~TIM_CCMR2_OC3PE;
  // Enable CH3 output
  TIM20->CCER |= TIM_CCER_CC3E;
  // Update registers
  TIM20->EGR |= TIM_EGR_UG;
  // Enable Counter
  TIM20->CR1 |= TIM_CR1_CEN;
  TIM20->BDTR |= TIM_BDTR_MOE;
}
