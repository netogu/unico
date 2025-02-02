#ifndef __STM32G4_QENC_H
#define __STM32G4_QENC_H
#include <stdint.h>

typedef struct {
  void *timer;
  enum {
    QENC_MODE_ABZ,
    QENC_MODE_TIMER,
  } mode;

  uint32_t cpr;

} qenc_t;

int qenc_init(qenc_t *self);
void qenc_run(qenc_t *self);
void qenc_stop(qenc_t *self);
void qenc_reset(qenc_t *self);
void qenc_load(qenc_t *self, uint32_t count);
uint32_t qenc_read(qenc_t *self);
int qenc_timer_set_frequency(qenc_t *self, uint32_t hz);

#endif // stm32g4_encoder.h
