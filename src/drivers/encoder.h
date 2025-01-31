#ifndef ENCODER_H
#define ENCODER_H

/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include <stdint.h>

typedef struct {
  uint32_t count;
  uint32_t cpr;
} encoder_t;

int encoder_init(encoder_t *self, uint32_t count_per_rev);
void encoder_update(encoder_t *self);
void encoder_load_count(encoder_t *self, uint32_t value);

#endif // encoder.h
