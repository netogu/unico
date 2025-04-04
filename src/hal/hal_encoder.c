/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include "hal.h"
#include <stdint.h>
#include <string.h>

int hal_encoder_init(hal_encoder_t *self, hal_encoder_ops_t ops) {
  if (self == NULL) {
    return -1;
  }
  memset(self, 0, sizeof(hal_encoder_t));

  self->ops = ops;
  self->cpr = self->ops.get_cpr();

  return 0;
}

int hal_encoder_update(hal_encoder_t *self) {
  if (self == NULL) {
    return -1;
  }
  // self->cpr = self->ops.get_cpr();
  self->ops.update();
  return 0;
}

int32_t hal_encoder_read_count(hal_encoder_t *self) { return self->ops.read(); }

void hal_encoder_set_offset(hal_encoder_t *self, uint32_t offset) {
  self->ops.set_offset(offset);
}

int32_t hal_encoder_read_angle_q31(hal_encoder_t *self) {
  int32_t cpr = self->cpr;
  int32_t total_count = self->ops.read();
  int32_t pos = (total_count % cpr);
  pos = (pos + (cpr / 2)) % cpr;

  // Maping from [0, 2Pi] to [-Pi, Pi]
  // normalized to [0, 2.0] and [-1.0, 1.0 - 2^(-31) ]
  int32_t angle_q31 = ((int64_t)pos * (1LL << 32) / cpr) - (1 << 31);
  return angle_q31;
}
