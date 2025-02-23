#ifndef STM32G4_ADC_H
#define STM32G4_ADC_H

#include "hal_analog.h"
#include "stm32g4_common.h"

#define ADC_REG_INPUT_MAX 16
#define ADC_INJ_INPUT_MAX 4

typedef struct adc_s {
  struct {
    hal_analog_input_t *input;
    uint16_t sample_time;
  } regular_inputs[ADC_REG_INPUT_MAX];

  struct {
    hal_analog_input_t *input;
    uint16_t sample_time;
  } injected_inputs[ADC_INJ_INPUT_MAX];

  uint16_t num_regular_inputs;
  uint16_t num_injected_inputs;

  void *regs;
  void *options;

} adc_t;

int adc_init(adc_t *self);
int adc_enable(adc_t *self);
int adc_register_input(adc_t *self, hal_analog_input_t *input, char input_type,
                       uint16_t sample_time);
int adc_enable_injected_input_soc_trigger(adc_t *self);

int adc_start_regular_sampling(adc_t *self);
int adc_start_injected_sampling(adc_t *self);
int adc_stop_regular_sampling(adc_t *self);
int adc_stop_injected_sampling(adc_t *self);

#endif
