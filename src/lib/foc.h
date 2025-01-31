#ifndef _FOC_H
#define _FOC_H

#include "stdint.h"

typedef struct {
  float kp;
  float ki;
  float integral;
  float setpoint;
  float output;
} pi_controller_t;

typedef struct {
  float R;      // Phase resistance
  float L;      // Phase inductance
  float lambda; // Flux linkage
  uint32_t pole_pair;
} motor_t;

typedef struct {

  enum {
    FOC_STATUS_OFF,
    FOC_STATUS_IDLE,
    FOC_STATUS_RUN_VOLTAGE,
    FOC_STATUS_RUN_CURRENT,
    FOC_STATUS_RUN_POSITION,
    FOC_STATUS_RUN_VELOCITY,
    FOC_STATUS_FAULTED,
    FOC_STATUS_ERROR,
  } status;

  float theta;      // Rotor Angle (rad) (electrical)
  float ia, ib, ic; // Phase currents (measured)
  float id, iq;     // q-q axist currents
  float vd, vq;     // d-q axis voltages
  float vbus;       // Motor bus voltage
  float temp_a;     // Power stage temperature A
  float temp_b;     // Power stage temperature B
  float temp_c;     // Power stage temperature C
  float temp_m;     // Motor temperature

  const motor_t motor;
  pi_controller_t pi_id;
  pi_controller_t pi_iq;

} foc_t;

int foc_init(foc_t *self);
int foc_update(foc_t *self);
int foc_set_current(foc_t *self, float id, float iq);
int foc_set_position(foc_t *self, float theta);
int foc_set_speed(foc_t *self, float omega);
int foc_run(foc_t *self);
int foc_stop(foc_t *self);

#endif // foc.h
