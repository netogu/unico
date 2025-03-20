

//--------------------------------------------------------------------+
// PWM Control Task
//--------------------------------------------------------------------+
#include "bsp.h"
#include "hal.h"
#include "hal_stm32_cordic.h"
#include "lib/motor_control.h"
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
  foc->sp.id_sp = (float)msg->id_ma / 1000.0f; // Convert to amps from mA
  foc->sp.iq_sp = (float)msg->iq_ma / 1000.0f; // Convert to amps from mA
  foc->mode = msg->mode;
  // foc->_int.count_rate = msg->count_rate;

  return 0;
}

static void task_pwmcon_timer_callback() {
  // Timer Callback
  board_t *brd = board_get_handle();
  // Read Feedback
  foc.rotor_angle_q31 = hal_encoder_read_angle_q31(&brd->hw.encoder);
  foc.abc.iphase[0] = hal_analog_read(&brd->ai.ia_fb);
  foc.abc.iphase[1] = hal_analog_read(&brd->ai.ib_fb);
  foc.abc.iphase[2] = hal_analog_read(&brd->ai.ic_fb);
  foc.vbus = hal_analog_read(&brd->ai.vm_fb);
  // Calculate new PWM
  moc_foc_update(&foc);
  // Update PWM
  hal_pwm_3ph_set_duty_f32(&brd->hw.mcpwm, foc.pwm.duty);
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
  float ia = hal_analog_read(&brd->ai.ia_fb);
  float ib = hal_analog_read(&brd->ai.ib_fb);
  float ic = hal_analog_read(&brd->ai.ic_fb);
  brd->ai.ia_fb.offset = -ia;
  brd->ai.ib_fb.offset = -ib;
  brd->ai.ic_fb.offset = -ic;

  moc_foc_init(&foc);
  moc_foc_update(&foc);

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
