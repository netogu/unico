
#include "hal.h"

inline uint32_t hal_analog_read_raw(hal_analog_input_t *self) {
  return *self->data;
}

inline float hal_analog_read_f32(hal_analog_input_t *self) {
  float value = *self->data * self->scale + self->offset;
  return value;
}
