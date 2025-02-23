#ifndef __HAL_ANAlOG__
#define __HAL_ANAlOG__

#include <stdint.h>

typedef struct {
  char *name;
  uint8_t channel;
  float scale;
  float offset;
  char *units;
  volatile uint32_t *data;
} hal_analog_input_t;

uint32_t hal_analog_read_raw(hal_analog_input_t *self);
float hal_analog_read_f32(hal_analog_input_t *self);

#endif
