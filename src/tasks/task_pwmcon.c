

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+

#include "arm_math.h"
#include "bsp.h"
#include "encoder.h"
#include "hal.h"
#include "lib/foc.h"
#include "math.h"
#include "tasklist.h"
#include "taskmsg.h"

static TaskHandle_t task_pwm_control_handle;
static StaticTask_t task_pwm_control_tcb;
static StackType_t task_pwm_control_stack[TASK_STACK_SIZE_PWM_CONTROL];
static void task_pwm_control_app(void *parameters);

enum pwmcon_encoder_type {
  PWMCON_ENCODER_TYPE_SW,
  PWMCON_ENCODER_TYPE_DIRECT_PWM,
  PWMCON_ENCODER_TYPE_ABZ,
};

static foc_t foc = {0};
static encoder_t sw_enc = {.count = 0, .cpr = 10000};
static uint32_t sw_enc_count_rate;
static enum pwmcon_encoder_type encoder_type = PWMCON_ENCODER_TYPE_ABZ;

static pwmcon_msg_t msg = {0};
static void task_pwmcon_timer_callback(void);
static float fast_fmodf(float x, float y);

TaskHandle_t task_pwm_control_init(void) {
  // Create the task

  task_pwm_control_handle = xTaskCreateStatic(
      task_pwm_control_app, xstr(TASK_NAME_PWM_CONTROL),
      TASK_STACK_SIZE_PWM_CONTROL, NULL, TASK_PRIORITY_PWM_CONTROL,
      task_pwm_control_stack, &task_pwm_control_tcb);

  if (task_pwm_control_handle == NULL) {
    return NULL;
  }
  return task_pwm_control_handle;
}

static inline void task_pwmcon_notify(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveIndexedFromISR(task_pwm_control_handle, 0,
                                &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static inline int pwmcon_foc_process_message(foc_t *foc, pwmcon_msg_t *msg) {

  board_t *brd = board_get_handle();

  foc->vbus = (float)(msg->vbus_mv + 1) / 1000.0f; // Convert to volts from mV
  foc->vq = (float)msg->vq_mv / 1000.0f;           // Convert to volts from mV
  foc->vd = (float)msg->vd_mv / 1000.0f;           // Convert to volts from mV
  sw_enc_count_rate = msg->count_rate;

  if (msg->mode == PWMCON_ENCODER_TYPE_ABZ) {
    if (encoder_type != PWMCON_ENCODER_TYPE_ABZ) {
      uint32_t count = (sw_enc.count * brd->hw.menc.cpr) / sw_enc.cpr;
      encoder_load_count(&brd->hw.menc, count);
      // Update FOC angle
      encoder_type = PWMCON_ENCODER_TYPE_ABZ;
    }

  } else

      if (msg->mode == PWMCON_ENCODER_TYPE_SW) {
    if (encoder_type != PWMCON_ENCODER_TYPE_SW) {
      encoder_update(&brd->hw.menc);
      sw_enc.count = (brd->hw.menc.count * sw_enc.cpr) / brd->hw.menc.cpr;
      // Update FOC angle
      encoder_type = PWMCON_ENCODER_TYPE_SW;
    }
  } else if (msg->mode == PWMCON_ENCODER_TYPE_DIRECT_PWM) {
    uint32_t mcpwm_duty[3];
    for (int i = 0; i < 3; i++) {
      mcpwm_duty[i] = (float)msg->duty_cmd[i] / 100.0f;
    }
    pwm_3ph_set_duty(&brd->hw.mcpwm, mcpwm_duty[0], mcpwm_duty[1],
                     mcpwm_duty[2]);
    encoder_type = PWMCON_ENCODER_TYPE_DIRECT_PWM;

  } else {
    cli_printf("PWMCON Message Error\r\n");
    return -1;
  }

  return 0;
}

static void pwmcon_foc_update(foc_t *foc) {
  // Timer Callback

  board_t *brd = board_get_handle();
  float angle_rad = 0.0;

  // Get Angle
  switch (encoder_type) {
  case PWMCON_ENCODER_TYPE_ABZ:
    encoder_update(&brd->hw.menc);
    angle_rad = (float)brd->hw.menc.count / brd->hw.menc.cpr * 2 * PI;
    break;
  case PWMCON_ENCODER_TYPE_SW:
    sw_enc.count += sw_enc_count_rate;
    if (sw_enc.count > sw_enc.cpr) {
      sw_enc.count = 0;
    }
    angle_rad = (float)sw_enc.count / sw_enc.cpr * 2 * PI;
    break;
  case PWMCON_ENCODER_TYPE_DIRECT_PWM:
    // No FOC needed
    return;
    break;
  default:
    break;
  }

  float angle_rad_norm = fast_fmodf(angle_rad, 2.0f * PI) / (2.0f * PI);
  int32_t angle_rad_q31 = f32_to_q31(angle_rad_norm) << 1;

  cordic_write(angle_rad_q31);
  int32_t cos_q31 = cordic_read();
  int32_t sin_q31 = cordic_read();

  float cos_f32 = q31_to_f32(cos_q31);
  float sin_f32 = q31_to_f32(sin_q31);

  float vd_norm = foc->vd / foc->vbus;
  float vq_norm = foc->vq / foc->vbus;
  float valpha, vbeta;
  float vphase[3];
  float duty[3];

  arm_inv_park_f32(vd_norm, vq_norm, &valpha, &vbeta, sin_f32, cos_f32);
  arm_inv_clarke_f32(valpha, vbeta, &vphase[0], &vphase[1]);

  // a + b + c = 0
  vphase[2] = -vphase[0] - vphase[1];

  for (int i = 0; i < 3; i++) {
    duty[i] = (vphase[i] + 1.0f) / 2.0f;
  }

  pwm_3ph_set_duty(&brd->hw.mcpwm, duty[0], duty[1], duty[2]);
}

static void task_pwmcon_timer_callback() {
  // Timer Callback
  pwmcon_foc_update(&foc);
}

static void task_pwm_control_app(void *parameters) {

  // board_t *brd = board_get_handle();
  if (foc.vbus <= 0.0f) {
    foc.vbus = 0.001;
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

  foc.vbus = 0.001;

  while (1) {

    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // ulTaskNotifyTakeIndexed(0, pdTRUE, portMAX_DELAY);
    // gpio_pin_set(&brd->io.test_pin0);

    // Check for new messages
    if (task_pwmcon_msg_receive(&msg)) {
      taskENTER_CRITICAL();
      pwmcon_foc_process_message(&foc, &msg);
      taskEXIT_CRITICAL();
    }
    // task_pwmcon_timer_callback();

    // gpio_pin_clear(&brd->io.test_pin0);
    vTaskDelay(TASK_DELAY_PWM_CONTROL);
  }
}

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
