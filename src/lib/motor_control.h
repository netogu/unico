#ifndef __MOC_H__
#define __MOC_H__

#include "dsp/controller_functions.h"
#include <stdint.h>

enum moc_foc_mode {
  FOC_MODE_OPEN,
  FOC_MODE_VOLTAGE_CONTROL,
  FOC_MODE_CURRENT_CONTROL,
  FOC_MODE_PHASE_LOCK,
  FOC_MODE_3PHASE_SHORT,
};

typedef struct {

  enum moc_foc_mode mode;

  enum {
    FOC_PHASE_ABC,
    FOC_PHASE_ACB,
  } phase_mode;

  arm_pid_instance_f32 pid_id;
  arm_pid_instance_f32 pid_iq;

  float vbus;
  float rotor_angle_f32;
  int32_t rotor_angle_q31;

  struct {
    float id_sp;
    float iq_sp;
    float vq_sp;
    float vd_sp;
  } sp;

  struct {
    float vq;
    float vd;
    float iq;
    float id;
    float vq_lim;
    float vd_lim;
    float iq_lim;
    float id_lim;
  } dq0;

  struct {
    float iphase[3];
    float vphase[3];
  } abc;

  struct {
    float duty[3];
  } pwm;

} moc_foc_t;

void moc_foc_init(moc_foc_t *self);
void moc_foc_update(moc_foc_t *self);
void moc_foc_set_mode(moc_foc_t *self, enum moc_foc_mode mode);
void moc_foc_pid_update(moc_foc_t *self, uint32_t reset);

#endif
