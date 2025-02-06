/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include "hal_encoder.h"
#include <stdint.h>
#include <string.h>

int encoder_init(encoder_t *self, encoder_ops_t ops) {
  if (self == NULL) {
    return -1;
  }
  memset(self, 0, sizeof(encoder_t));

  self->ops = ops;
  self->cpr = self->ops.get_cpr();

  return 0;
}

int encoder_update(encoder_t *self) {
  if (self == NULL) {
    return -1;
  }

  // self->cpr = self->ops.get_cpr();
  self->ops.update();
  self->count = self->ops.read();

  return 0;
}
int32_t encoder_read_count(encoder_t *self) { return self->count; }
void encoder_set_offset(encoder_t *self, uint32_t offset) {
  self->ops.set_offset(offset);
}
int32_t encoder_read_angle_q31(encoder_t *self) {
  int32_t angle_q31;
  angle_q31 = ((int64_t)self->count * 0x7FFFFFFF) / self->cpr;
  angle_q31 -= 0x40000000;
  return angle_q31;
}
float encoder_read_angle_f32(encoder_t *self);
