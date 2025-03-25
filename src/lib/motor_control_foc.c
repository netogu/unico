#include "dsp/controller_functions.h"
#include "hal_stm32_cordic.h"
#include "motor_control.h"
#include <stdint.h>

arm_pid_instance_f32 pid_id;
arm_pid_instance_f32 pid_iq;

__attribute__((always_inline)) static inline float
moc_saturate_symetric_f32(float in, float lim) {
  // float out = in;
  // if (in > lim) {
  //   out = lim;
  // } else if (in < -lim) {
  //   out = -lim;
  // }
  // return out;
  return fmaxf(-lim, fminf(in, lim));
}

inline void moc_foc_pid_update(moc_foc_t *self, uint32_t reset) {

  arm_pid_init_f32(&self->pid_id, reset);
  arm_pid_init_f32(&self->pid_iq, reset);
}

void moc_foc_init(moc_foc_t *self) {

  self->pid_id.Kp = 6.0;
  self->pid_id.Ki = 0.001;
  self->pid_id.Kd = 0.0;

  self->pid_iq.Kp = 6.0;
  self->pid_iq.Ki = 0.001;
  self->pid_iq.Kd = 0.0;

  self->dq0.vd_lim = 10.0f;
  self->dq0.vq_lim = 10.0f;

  self->dq0.id_lim = 15.0;
  self->dq0.iq_lim = 15.0;

  moc_foc_pid_update(self, 1);
}

__attribute__((always_inline)) inline void moc_foc_update(moc_foc_t *self) {
  if (self == NULL) {
    return;
  }

  float ialpha = 0;
  float ibeta = 0;
  float valpha = 0;
  float vbeta = 0;
  float vd_norm = 0;
  float vq_norm = 0;

  // Calculate Sin/Cos of Theta_e
  if (self->mode == FOC_MODE_PHASE_LOCK) {
    cordic_write(0);
  } else {
    cordic_write(self->rotor_angle_q31);
  }

  int32_t cos_q31 = cordic_read();
  int32_t sin_q31 = cordic_read();
  float cos_f32 = q31_to_f32(cos_q31);
  float sin_f32 = q31_to_f32(sin_q31);

  if (self->mode == FOC_MODE_CURRENT_CONTROL) {
    arm_clarke_f32(self->abc.iphase[0], self->abc.iphase[2], &ialpha, &ibeta);
    arm_park_f32(ialpha, ibeta, &self->dq0.id, &self->dq0.iq, sin_f32, cos_f32);

    // (PID iD, iQ) --> Vd, Vq
    float id_error = self->sp.id_sp - self->dq0.id;
    float iq_error = self->sp.iq_sp - self->dq0.iq;

    self->dq0.vd = arm_pid_f32(&self->pid_id, id_error);
    self->dq0.vq = arm_pid_f32(&self->pid_iq, iq_error);

    // Saturate vd, vq
    self->dq0.vd = moc_saturate_symetric_f32(self->dq0.vd, self->dq0.vd_lim);
    self->dq0.vq = moc_saturate_symetric_f32(self->dq0.vq, self->dq0.vq_lim);

    vd_norm = self->dq0.vd / self->vbus;
    vq_norm = self->dq0.vq / self->vbus;

  } else {

    vq_norm = self->sp.vq_sp / self->vbus;
    vd_norm = self->sp.vd_sp / self->vbus;
  }

  arm_inv_park_f32(vd_norm, vq_norm, &valpha, &vbeta, sin_f32, cos_f32);
  arm_inv_clarke_f32(valpha, vbeta, &self->abc.vphase[0], &self->abc.vphase[2]);

  // a + b + c = 0
  self->abc.vphase[1] = -self->abc.vphase[0] - self->abc.vphase[2];

  for (int i = 0; i < 3; i++) {
    // Map from (-1.0, 1.0) to (0.0, 1.0)
    self->pwm.duty[i] = (self->abc.vphase[i] + 1.0f) / 2.0f;
  }
}

void moc_foc_set_mode(moc_foc_t *self, enum moc_foc_mode mode) {
  self->mode = mode;
}
