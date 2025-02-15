

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+

#include "arm_math.h"
#include "bsp.h"
#include "hal.h"
#include "hal_encoder.h"
#include "tasklist.h"
#include "taskmsg.h"
#include <stdint.h>

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

typedef struct pwmcon_foc_t {
  struct {
    float vbus;
    float angle_rad;
    float iphase[3];
  } fb;
  struct {
    float vq;
    float vd;
  } dq0;

  struct {
    float vphase[3];
  } abc;

  struct {
    float duty[3];
  } pwm;

  struct {
    float cos_f32;
    float sin_f32;
    float valpha;
    float vbeta;
    uint32_t count;
    uint32_t count_rate;
    enum {
      PWMCON_FOC_MODE_OPEN_LOOP,
      PWMCON_FOC_MODE_MANUAL,
      PWMCON_FOC_MODE_FORCE_PWM,
    } mode;
  } _int;
} pwmcon_foc_t;

static pwmcon_foc_t foc = {0};
static pwmcon_msg_t msg = {0};
static void task_pwmcon_timer_callback(void);

static TaskHandle_t task_pwm_control_handle;
static StaticTask_t task_pwm_control_tcb;
static StackType_t task_pwm_control_stack[TASK_STACK_SIZE_PWM_CONTROL];
static void task_pwm_control(void *parameters);

TaskHandle_t task_pwm_control_init(void) {
  // Create the task

  task_pwm_control_handle = xTaskCreateStatic(
      task_pwm_control, xstr(TASK_NAME_PWM_CONTROL),
      TASK_STACK_SIZE_PWM_CONTROL, NULL, TASK_PRIORITY_PWM_CONTROL,
      task_pwm_control_stack, &task_pwm_control_tcb);

  if (task_pwm_control_handle != NULL) {
    return task_pwm_control_handle;
  } else {
    return NULL;
  }
}

static inline int pwmcon_foc_process_message(pwmcon_foc_t *foc,
                                             pwmcon_msg_t *msg) {

  foc->fb.vbus =
      (float)(msg->vbus_mv + 1) / 1000.0f;   // Convert to volts from mV
  foc->dq0.vq = (float)msg->vq_mv / 1000.0f; // Convert to volts from mV
  foc->dq0.vd = (float)msg->vd_mv / 1000.0f; // Convert to volts from mV
  foc->_int.count_rate = msg->count_rate;

  return 0;
}

static void pwmcon_foc_update(pwmcon_foc_t *foc) {
  // Timer Callback

  board_t *brd = board_get_handle();

  int32_t angle_rad_q31 = encoder_read_angle_q31(&brd->hw.encoder);

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
    foc->pwm.duty[i] = (foc->abc.vphase[i] + 1.0f) /
                       2.0f; // Map from (-1.0, 1.0) to (0.0, 1.0)
  }

  pwm_3ph_set_duty(&brd->hw.mcpwm, foc->pwm.duty[0], foc->pwm.duty[1],
                   foc->pwm.duty[2]);
}

static void task_pwmcon_timer_callback() {
  // Timer Callback
  pwmcon_foc_update(&foc);
}

static void task_pwm_control(void *parameters) {

  // board_t *brd = board_get_handle();
  if (foc.fb.vbus <= 0.0f) {
    foc.fb.vbus = 0.001;
  }

  /* Unused parameters. */
  (void)parameters;

  cordic_t cordic = {
      .function = CORDIC_FUNC_COSINE,
      .cycles = 6,
  };

  cordic_init(&cordic);

  // Create and start Task Timer
  hal_timer_t *task_pwmcon_timer =
      timer_create(2000, task_pwmcon_timer_callback);
  if (task_pwmcon_timer) {
    timer_start(task_pwmcon_timer);
  } else {
    cli_printf("ERROR:\tTimer Failed to Start\r\n");
  }

  foc.fb.vbus = 1.0;

  while (1) {

    // Check for new messages
    if (task_pwmcon_msg_receive(&msg)) {
      taskENTER_CRITICAL();
      pwmcon_foc_process_message(&foc, &msg);
      taskEXIT_CRITICAL();
    }

    vTaskDelay(TASK_DELAY_PWM_CONTROL);
  }
}
