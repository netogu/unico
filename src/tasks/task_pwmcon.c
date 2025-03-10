

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+
#include "bsp.h"
#include "hal.h"
#include "lib/moc.h"
#include "stm32g4_adc.h"
#include "tasklist.h"
#include "taskmsg.h"

static moc_foc_t foc = {0};
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

static inline int pwmcon_foc_process_message(moc_foc_t *foc,
                                             pwmcon_msg_t *msg) {

  foc->sp.vd_sp = (float)msg->vd_mv / 1000.0f; // Convert to volts from mV
  foc->sp.vq_sp = (float)msg->vq_mv / 1000.0f; // Convert to volts from mV
  foc->mode = msg->mode;
  // foc->_int.count_rate = msg->count_rate;

  return 0;
}

static void task_pwmcon_timer_callback() {
  // Timer Callback
  board_t *brd = board_get_handle();
  // Read Feedback
  foc.rotor_angle_q31 = encoder_read_angle_q31(&brd->hw.encoder);
  foc.abc.iphase[0] = adc_read_value_f32(&brd->ai.ia_fb);
  foc.abc.iphase[1] = adc_read_value_f32(&brd->ai.ib_fb);
  foc.abc.iphase[2] = adc_read_value_f32(&brd->ai.ic_fb);
  foc.vbus = adc_read_value_f32(&brd->ai.vm_fb);
  // Calculate new PWM
  moc_foc_update(&foc);
  // Update PWM
  pwm_3ph_set_duty(&brd->hw.mcpwm, foc.pwm.duty[0], foc.pwm.duty[1],
                   foc.pwm.duty[2]);
}

static void task_pwm_control(void *parameters) {

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
      timer_create(1000, task_pwmcon_timer_callback);
  if (task_pwmcon_timer) {
    timer_start(task_pwmcon_timer);
  } else {
    cli_printf("ERROR:\tTimer Failed to Start\r\n");
  }

  foc.vbus = 1.0;

  // Zero Currents
  board_t *brd = board_get_handle();
  float ia = adc_read_value_f32(&brd->ai.ia_fb);
  float ib = adc_read_value_f32(&brd->ai.ib_fb);
  float ic = adc_read_value_f32(&brd->ai.ic_fb);
  brd->ai.ia_fb.offset = -ia;
  brd->ai.ib_fb.offset = -ib;
  brd->ai.ic_fb.offset = -ic;

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
