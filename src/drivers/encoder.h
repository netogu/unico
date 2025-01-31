#ifndef ENCODER_H
#define ENCODER_H

/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include <stdint.h>

typedef struct {
  volatile uint32_t *count;
  uint32_t count_per_rev;
} encoder_t;

int encoder_init(encoder_t *self, uint32_t count_per_rev);
uint32_t encoder_read_count(encoder_t *self);
uint32_t encoder_get_cpr(encoder_t *self);
void encoder_load_count(encoder_t *self, uint32_t value);
uint32_t encoder_read_angle_q31(encoder_t *self);
float encoder_read_angle_float(encoder_t *self);

#endif // encoder.h
