#ifndef ENCODER_H
#define ENCODER_H

/*--------------------------------------------------*/
/* Encoders                                         */
/*--------------------------------------------------*/

#include <stdint.h>

typedef struct {
  uint32_t (*read)(void);
  void (*set_offset)(uint32_t offset);
  uint32_t (*get_cpr)(void);
  void (*update)(void);

} encoder_ops_t;

typedef struct {
  encoder_ops_t ops;
  int32_t pos;
  uint32_t count;
  uint32_t cpr;

} encoder_t;

int encoder_init(encoder_t *self, encoder_ops_t ops);
int encoder_update(encoder_t *self);
uint32_t encoder_read_count(encoder_t *self);
void encoder_set_offset(encoder_t *self, uint32_t count);
int32_t encoder_read_angle_q31(encoder_t *self);
float encoder_read_angle_f32(encoder_t *self);

#endif
