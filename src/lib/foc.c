#include "foc.h"
#include "bsp.h"
#include "encoder.h"
#include <string.h>

#define ANGLE_CORDIC                                                           \
  (int32_t)0x10000000               /* pi/8 in CORDIC input angle mapping */
#define MODULUS (int32_t)0x7FFFFFFF /* 1 */
#define COS_REF (int32_t)0x7641AF3C /* cos(pi/8) reference value */
#define SIN_REF (int32_t)0x30FBC54D /* sin(pi/8) reference value */
#define DELTA                                                                  \
  (int32_t)0x00001000 /* Max residual error for cos and sin, with 6 cycle      \
                         precision: 2^-19 max residual error, ie 31-19=12 LSB, \
                         ie <0x1000 */
#define COS_120D (float)-0.50
#define SIN_120D (float)+0.866025404
#define COS_240D COS_120D
#define SIN_240D -SIN_120D

int foc_init(foc_t *self) {
  memset(self, 0, sizeof(foc_t));
  return 0;
}

static int foc_clarke_transform(foc_t *self);
static int foc_park_transform(foc_t *self);
static int foc_inverse_park_transform(foc_t *self);
static float fast_fmod(float x, float y);
static inline int foc_update_angle(foc_t *self, encoder_t *encoder) {
  uint32_t angle_raw = encoder->read_raw();

  cordic_write(angle_rad_q31);
  int32_t cos_q31 = cordic_read();
  int32_t sin_q31 = cordic_read();

  int foc_update(foc_t * self) {

    board_t *brd = board_get_handle();
    // gpio_pin_set(&brd->io.test_pin0);
    // task_pwmcon_notify();
    // // gpio_pin_clear(&brd->io.test_pin0);

    // Get Angle
    switch (foc->_int.mode) {
    case PWMCON_FOC_MODE_MANUAL:
      foc->_int.count = brd->hw.encoder.read();
      foc->fb.angle_rad = (float)foc->_int.count / 1024.0f * 2 * PI;
      break;
    case PWMCON_FOC_MODE_OPEN_LOOP:
      foc->_int.count += foc->_int.count_rate;
      if (foc->_int.count > 100000) {
        foc->_int.count = 0;
      }
      foc->fb.angle_rad = (float)foc->_int.count / 100000.0f * 2 * PI;
      break;
    case PWMCON_FOC_MODE_FORCE_PWM:
      // No FOC needed
      return;
      break;
    default:
      break;
    }

    float angle_rad_norm =
        fast_fmodf(foc->fb.angle_rad, 2.0f * PI) / (2.0f * PI);
    int32_t angle_rad_q31 = f32_to_q31(angle_rad_norm) << 1;

    cordic_write(angle_rad_q31);
    int32_t cos_q31 = cordic_read();
    int32_t sin_q31 = cordic_read();

    foc->_int.cos_f32 = q31_to_f32(cos_q31);
    foc->_int.sin_f32 = q31_to_f32(sin_q31);

    float vd_norm = foc->dq0.vd / foc->fb.vbus;
    float vq_norm = foc->dq0.vq / foc->fb.vbus;
    float valpha, vbeta;

    arm_inv_park_f32(vd_norm, vq_norm, &valpha, &vbeta, foc->_int.sin_f32,
                     foc->_int.cos_f32);
    arm_inv_clarke_f32(valpha, vbeta, &foc->abc.vphase[0], &foc->abc.vphase[1]);

    // a + b + c = 0
    foc->abc.vphase[2] = -foc->abc.vphase[0] - foc->abc.vphase[1];

    for (int i = 0; i < 3; i++) {
      foc->pwm.duty[i] = (foc->abc.vphase[i] + 1.0f) / 2.0f;
    }

    pwm_3ph_set_duty(&brd->hw.mcpwm, foc->pwm.duty[0], foc->pwm.duty[1],
                     foc->pwm.duty[2]);
  }
  int foc_set_current(foc_t * self, float id, float iq);
  int foc_set_position(foc_t * self, float theta);
  int foc_set_speed(foc_t * self, float omega);
  int foc_run(foc_t * self);
  int foc_stop(foc_t * self);

  static float fast_fmodf(float x, float y) {
    if (y == 0.0f) {
      return NAN; // Return NaN for undefined behavior
    }

    // Calculate the integer multiple of y closest to x
    float quotient = (int)(x / y); // Cast to int truncates toward zero
    float result = x - quotient * y;

    // Adjust result if it goes out of range due to truncation
    if (result < 0.0f && y > 0.0f) {
      result += y;
    } else if (result > 0.0f && y < 0.0f) {
      result += y;
    }

    return result;
  }
