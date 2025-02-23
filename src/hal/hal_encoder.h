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

} hal_encoder_ops_t;

typedef struct {
  hal_encoder_ops_t ops;
  uint32_t cpr;

} hal_encoder_t;

int hal_encoder_init(hal_encoder_t *self, hal_encoder_ops_t ops);
int hal_encoder_update(hal_encoder_t *self);
int32_t hal_encoder_read_count(hal_encoder_t *self);
void hal_encoder_set_offset(hal_encoder_t *self, uint32_t count);
int32_t hal_encoder_read_angle_q31(hal_encoder_t *self);
float hal_encoder_read_angle_f32(hal_encoder_t *self);

#endif
