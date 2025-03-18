
#include "hal.h"

static inline uint32_t hal_analog_read_u32(void *self) {
  hal_analog_input_t *s = (hal_analog_input_t *)self;

  return *s->data;
}

static inline float hal_analog_read_f32(void *self) {
  hal_analog_input_t *s = (hal_analog_input_t *)self;
  float value = *s->data * s->scale + s->offset;
  return value;
}

int hal_analog_input_init(hal_analog_input_t *self) {
  if (!self) {
    return -1;
  }

  self->read_f32 = &hal_analog_read_f32;
  self->read_u32 = &hal_analog_read_u32;

  return 0;
}
