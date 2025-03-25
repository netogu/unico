#include "shell.h"
#include "bsp.h"
#include "hal.h"
#include "hal_stm32_cordic.h"
#include "rtos.h"
#include "sensors/sensor_ntc.h"
#include "taskmsg.h"
#include "tusb.h"
#include "uclib.h"

#define NOCHAR '\0'
#define SHELL_HOSTNAME "motor"

extern void shell_cmd_top_exec_cb(struct ush_object *self,
                                  struct ush_file_descriptor const *file,
                                  int argc, char *argv[]);

// non-blocking read interface
static int ush_read(struct ush_object *self, char *ch) {
  (void)self;

  char readchar = NOCHAR;

#ifdef SHELL_INTERFACE_USB
  if (tud_cdc_connected() && tud_cdc_available() > 0) {
    // Write single byte if mutex is available
    readchar = cli_usb_getc();
  }
#else
  readchar = cli_uart_getc();
#endif

  if (readchar != NOCHAR) {
    *ch = readchar;
    return 1;
  }
  return 0;
}

// non-blocking write interface
static int ush_write(struct ush_object *self, char ch) {
  (void)self;

#ifdef SHELL_INTERFACE_USB
  return cli_usb_putc(ch);
#else
  return cli_uart_putc(ch);
#endif
}

// I/O interface descriptor
static const struct ush_io_interface ush_iface = {
    .read = ush_read,
    .write = ush_write,
};

// working buffers allocations (size could be customized)
#define BUF_IN_SIZE 256
#define BUF_OUT_SIZE 256
#define PATH_MAX_SIZE 256

static char ush_in_buf[BUF_IN_SIZE];
static char ush_out_buf[BUF_OUT_SIZE];

// microshell instance handler
static struct ush_object ush;

// microshell descriptor
static const struct ush_descriptor ush_desc = {
    .io = &ush_iface,                          // I/O interface pointer
    .input_buffer = ush_in_buf,                // working input buffer
    .input_buffer_size = sizeof(ush_in_buf),   // working input buffer size
    .output_buffer = ush_out_buf,              // working output buffer
    .output_buffer_size = sizeof(ush_out_buf), // working output buffer size
    .path_max_length = PATH_MAX_SIZE,          // path maximum length (stack)
    .hostname = SHELL_HOSTNAME,                // hostname (in prompt)
};

// pint ADC helper functions

struct measurement {
  hal_analog_input_t ain;
  enum {
    INT32,
    F32,
    Q31,
  } type;
  uint32_t min;
  uint32_t max;
  uint32_t avg;
};

float adc_test_scale_meas_f32(uint32_t value_raw,
                              const struct measurement *meas) {
  return (float)value_raw * meas->ain.scale + meas->ain.offset;
}
// print ADC Callback
static void adc_test_read_callback(struct ush_object *self,
                                   struct ush_file_descriptor const *file,
                                   int argc, char *argv[]) {
  (void)file;
  (void)self;
  (void)argc;
  (void)argv;

  // arguments count validation
  // if (argc != 2) {
  //   // return predefined error message
  //   ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
  //   return;
  // }
  //
  // int samples = atoi(argv[1]);
  // if (samples < 0 || samples > 500000) {
  //   ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
  //   return;
  // }

  board_t *brd = board_get_handle();

  uint32_t enc_data = 0;
  int32_t angle_q31 = 0;
  hal_analog_input_t enc_raw = {.name = "enc", .data = &enc_data};
  hal_analog_input_t angle = {.name = "angle", .data = (uint32_t *)&angle_q31};

  struct measurement adc_list[13];
  adc_list[0] = (struct measurement){.ain = brd->ai.ia_fb, .type = F32};
  adc_list[1] = (struct measurement){.ain = brd->ai.ib_fb, .type = F32};
  adc_list[2] = (struct measurement){.ain = brd->ai.ic_fb, .type = F32};
  adc_list[3] = (struct measurement){.ain = brd->ai.vgd_mon, .type = F32};
  adc_list[4] = (struct measurement){.ain = brd->ai.vl_mon, .type = F32};
  adc_list[5] = (struct measurement){.ain = brd->ai.vbatt_mon, .type = F32};
  adc_list[6] = (struct measurement){.ain = brd->ai.vm_fb, .type = F32};
  adc_list[7] = (struct measurement){.ain = brd->ai.im_fb, .type = F32};
  adc_list[8] = (struct measurement){.ain = brd->ai.temp_a, .type = F32};
  adc_list[9] = (struct measurement){.ain = brd->ai.temp_b, .type = F32};
  adc_list[10] = (struct measurement){.ain = brd->ai.temp_c, .type = F32};
  adc_list[11] = (struct measurement){.ain = enc_raw, .type = INT32};
  adc_list[12] = (struct measurement){.ain = angle, .type = Q31};

  for (size_t i = 0; i < sizeof(adc_list) / sizeof(adc_list[0]); i++) {
    adc_list[i].min = 4096;
    adc_list[i].avg = 0;
    adc_list[i].max = 0;
  }

  uint32_t samples = 1;
  while (samples) {
    hal_encoder_update(&brd->hw.encoder);
    enc_data = hal_encoder_read_count(&brd->hw.encoder);
    angle_q31 = hal_encoder_read_angle_q31(&brd->hw.encoder);

    uint8_t c = cli_usb_getc();

    if (c == 0x03) {
      // Ctrl + C
      break;
    } else if (c == 'z') {
      // zero encoder
      hal_encoder_set_offset(&brd->hw.encoder, 0);
    }

    for (size_t i = 0; i < sizeof(adc_list) / sizeof(adc_list[0]); i++) {
      uint32_t value = *adc_list[i].ain.data;
      adc_list[i].min = (value < adc_list[i].min) ? value : adc_list[i].min;
      adc_list[i].max = (value > adc_list[i].max) ? value : adc_list[i].max;
      adc_list[i].avg += value;

      if (adc_list[i].type == F32) {
        char str_val[16];
        float value_f32 = hal_analog_read(&adc_list[i].ain);
        uclib_ftoa(value_f32, str_val, 2);
        cli_printf("%s=%s%s ", adc_list[i].ain.name, str_val,
                   adc_list[i].ain.units);
      } else if (adc_list[i].type == Q31) {

        char str_val[16];
        float value_f32 = q31_to_f32((int32_t)*adc_list[i].ain.data);
        uclib_ftoa(value_f32, str_val, 4);
        cli_printf("%s=%s%s ", adc_list[i].ain.name, str_val,
                   adc_list[i].ain.units);

      } else {
        cli_printf("%s=%04d ", adc_list[i].ain.name, value);
      }
    }

    // tud_cdc_write_flush();
    cli_printf("\n\r");

    samples++;
    vTaskDelay(1);
  }

  cli_printf("\n\rsamples: %ld\r\n", samples);
  for (size_t i = 0; i < sizeof(adc_list) / sizeof(adc_list[0]); i++) {
    adc_list[i].avg = adc_list[i].avg / samples;

    if (adc_list[i].type == F32) {
      char str_min[16];
      char str_max[16];
      char str_avg[16];
      float min_f32 = adc_test_scale_meas_f32(adc_list[i].min, &adc_list[i]);
      float max_f32 = adc_test_scale_meas_f32(adc_list[i].max, &adc_list[i]);
      float avg_f32 = adc_test_scale_meas_f32(adc_list[i].avg, &adc_list[i]);

      uclib_ftoa(min_f32, str_min, 2);
      uclib_ftoa(max_f32, str_max, 2);
      uclib_ftoa(avg_f32, str_avg, 2);

      cli_printf("%7s[%s]: min=%s avg=%s max=%s\r\n", adc_list[i].ain.name,
                 adc_list[i].ain.units, str_min, str_avg, str_max);

    } else {
      cli_printf("%7s[raw]: min=%04d avg=%04d max=%04d\r\n",
                 adc_list[i].ain.name, adc_list[i].min, adc_list[i].avg,
                 adc_list[i].max);
    }
    tud_cdc_write_flush();
    vTaskDelay(1);
  }

  return;
}

// drive enable callback
static void drv_en_callback(struct ush_object *self,
                            struct ush_file_descriptor const *file, int argc,
                            char *argv[]) {
  (void)self;
  (void)file;

  board_t *brd = board_get_handle();

  // arguments count validation
  if (argc != 2) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // arguments validation
  if (strcmp(argv[1], "1") == 0) {
    // turn gate driver on
    hal_gpio_set(&brd->dio.motor_en);
  } else if (strcmp(argv[1], "0") == 0) {
    // turn gate driver off
    hal_gpio_clear(&brd->dio.motor_en);
  } else {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }
}

// drive enable callback
static void mpwr_en_callback(struct ush_object *self,
                             struct ush_file_descriptor const *file, int argc,
                             char *argv[]) {

  (void)self; // unused
  (void)file; // unused

  board_t *brd = board_get_handle();

  // arguments count validation
  if (argc != 2) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // arguments validation
  if (strcmp(argv[1], "1") == 0) {
    // turn gate driver on
    hal_gpio_set(&brd->dio.mpwr_en);
  } else if (strcmp(argv[1], "0") == 0) {
    // turn VM efuse driver off
    hal_gpio_clear(&brd->dio.mpwr_en);
  } else {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }
}

// ocp threshold callback
static void ocp_exec_callback(struct ush_object *self,
                              struct ush_file_descriptor const *file, int argc,
                              char *argv[]) {
  (void)self; // unused
  (void)file; // unused

  // arguments count validation
  if (argc < 2) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  float threshold_pc = atoi(argv[1]) / 100.0;
  if (threshold_pc > 1.0f || threshold_pc < 0.0f) {
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // TIM20.3 at 50% duty cycle maps to the OCP threshold at 100%
  float duty = threshold_pc * 1 / 2.0f;
  duty = (duty > 0.5f) ? 0.5f : ((duty < 0.0f) ? 0.0f : duty);

  uint32_t period = TIM20->ARR;
  duty = period * duty + 0.5f;
  TIM20->CCR3 = (uint32_t)duty;
}

// dpt test callback
static void dpt_exec_callback(struct ush_object *self,
                              struct ush_file_descriptor const *file, int argc,
                              char *argv[]) {
  (void)self; // unused
  (void)file; // unused

  // board_t *brd = board_get_handle();

  // arguments count validation
  if (argc < 3) {
    // return predefined error message
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    return;
  }

  // hal_pwm_t *pwm = &brd->hw.mcpwm.pwm[0];

  int ton = atoi(argv[1]);
  ush_printf(self, "ton: %d ns\r\n", ton);
  int toff = atoi(argv[2]);
  ush_printf(self, "toff: %d ns\r\n", toff);
  int n = atoi(argv[3]);
  if (n <= 1) {
    ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
    ush_printf(self, "error: n has to be larger than 1\r\n");
    return;
  }
  ush_printf(self, "n: %d\r\n", n);

  // hrtim_pwm_stop(&pwm1);
  // get deadtime from HRTIM registers and convert to ns
  // uint32_t deadtime_ns = HRTIM1->sCommonRegs.DTR & HRTIM_DTR_DTF_Msk;
  uint32_t deadtime_ns = 0;

  ton += deadtime_ns;
  toff += deadtime_ns;

  ton < 0 ? ton = 0 : ton;
  toff < 0 ? toff = 0 : toff;

  // hal_pwm_set_frequency(pwm, 1000000000 / (ton + toff));
  // hal_pwm_set_duty(pwm, ton * 100 / (ton + toff));
  // hal_pwm_set_n_cycle_run(pwm, n);
  // hal_pwm_start(pwm);
}

static void _print_pwmcon_msg(struct ush_object *self, pwmcon_msg_t *msg) {
  ush_printf(self, "vq = %d mV\n\r", msg->vq_mv);
  ush_printf(self, "vd = %d mV\n\r", msg->vd_mv);
  ush_printf(self, "iq = %d mA\n\r", msg->iq_ma);
  ush_printf(self, "id = %d mA\n\r", msg->id_ma);
  ush_printf(self, "mode = %d\n\r", msg->mode);
}
// FOC Control Interface
static void foc_cmd_cb(struct ush_object *self,
                       struct ush_file_descriptor const *file, int argc,
                       char *argv[]) {
  (void)self; // unused
  (void)file; // unused

  board_t *brd = board_get_handle();
  (void)brd;

  static pwmcon_msg_t msg = {};

  // arguments count validation
  // if (argc < 1) {
  //     // return predefined error message
  //     ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
  //     return;
  // }

  if (argc == 1) {
    _print_pwmcon_msg(self, &msg);
    return;
  }

  int current_arg = 1;
  bool msg_updated = false;
  while (current_arg < argc) {
    if (strcmp(argv[current_arg], "vq") == 0) {
      int32_t vq = atoi(argv[current_arg + 1]);
      msg.vq_mv = vq;
      msg_updated = true;
      current_arg += 2;
    } else if (strcmp(argv[current_arg], "vd") == 0) {
      int32_t vd = atoi(argv[current_arg + 1]);
      msg.vd_mv = vd;
      msg_updated = true;
      current_arg += 2;
    } else if (strcmp(argv[current_arg], "iq") == 0) {
      int32_t iq = atoi(argv[current_arg + 1]);
      msg.iq_ma = iq;
      msg_updated = true;
      current_arg += 2;
    } else if (strcmp(argv[current_arg], "id") == 0) {
      int32_t id = atoi(argv[current_arg + 1]);
      msg.id_ma = id;
      msg_updated = true;
      current_arg += 2;
    } else if (strcmp(argv[current_arg], "mode") == 0) {
      msg.mode = atoi(argv[current_arg + 1]);
      msg_updated = true;
      current_arg += 2;
    } else if (strcmp(argv[current_arg], "phaselock") == 0) {
      msg.vd_mv = atoi(argv[current_arg + 1]);
      msg.mode = 3;
      task_pwmcon_msg_send(msg);
      vTaskDelay(500);
      hal_encoder_set_offset(&brd->hw.encoder, 0);
      msg.vd_mv = 0;
      msg.mode = 0;
      task_pwmcon_msg_send(msg);
      current_arg += 2;
    } else {
      ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
      return;
    }
  }

  if (msg_updated) {
    _print_pwmcon_msg(self, &msg);
    task_pwmcon_msg_send(msg);
  } else {
    // ush_print_status(self, USH_STATUS_ERROR_COMMAND_WRONG_ARGUMENTS);
  }

  return;
}

// reboot cmd file execute callback
static void reboot_exec_callback(struct ush_object *self,
                                 struct ush_file_descriptor const *file,
                                 int argc, char *argv[]) {
  (void)self; // unused
  (void)file; // unused
  (void)argc; // unused
  (void)argv; // unused

  NVIC_SystemReset();
}

// clear cmd file execute callback
static void clear_exec_callback(struct ush_object *self,
                                struct ush_file_descriptor const *file,
                                int argc, char *argv[]) {
  (void)file; // unused
  (void)argc; // unused
  (void)argv; // unused

  ush_print(self, "\033[2J\033[1;1H");
}

// info file get data callback
size_t info_get_data_callback(struct ush_object *self,
                              struct ush_file_descriptor const *file,
                              uint8_t **data) {
  (void)self; // unused
  (void)file; // unused

  static const char *info = "Minimal STM32G4 Shell\r\n";

  // return pointer to data
  *data = (uint8_t *)info;
  // return data size
  return strlen(info);
}

// ADC_DR file get data callback
size_t ADC_DR_get_data_callback(struct ush_object *self,
                                struct ush_file_descriptor const *file,
                                uint8_t **data) {
  (void)self; // unused
  (void)file; // unused

  board_t *brd = board_get_handle();

  static char dr[256];

  snprintf(dr, sizeof(dr), "vm = %ld\t ia = %ld\t ib = %ld\t ic =%ld\r\n",
           *brd->ai.vm_fb.data, *brd->ai.ia_fb.data, *brd->ai.ib_fb.data,
           *brd->ai.ic_fb.data);

  *data = (uint8_t *)dr;

  // return pointer to data
  // return data size
  return strlen(dr);
}

// root directory files descriptor
static const struct ush_file_descriptor root_files[] = {
    {
        .name = "info.txt", // info.txt file name
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = info_get_data_callback,
    },
    {
        .name = "ADC_DR", // info.txt file name
        .description = NULL,
        .help = NULL,
        .exec = NULL,
        .get_data = ADC_DR_get_data_callback,
    }};

// cmd files descriptor
static const struct ush_file_descriptor cmd_files[] = {
    {
        .name = "reboot",
        .description = "reboot device",
        .help = NULL,
        .exec = reboot_exec_callback,
    },
    {
        .name = "clear",
        .description = "clear terminal",
        .help = NULL,
        .exec = clear_exec_callback,
    },
    {
        .name = "top",
        .description = "Task Monitor",
        .help = NULL,
        .exec = shell_cmd_top_exec_cb,
    },
    {
        .name = "dpt",
        .description = "run double-pulse test", // optional file description
        .help = "usage: dpt channel(a,b,c) ton(ns) toff(ns) "
                "n(pulses)\r\n",   // optional help manual
        .exec = dpt_exec_callback, // optional execute callback
    },
    {
        .name = "drv_en",
        .description = "enable/disable gate driver",
        .help = "usage: drv_en 1|0\r\n",
        .exec = drv_en_callback,
    },
    {
        .name = "adc_test",
        .description = "Test phase current ADC measurement",
        .help = "usage: adc [samples]\r\n",
        .exec = adc_test_read_callback,
    },
    {
        .name = "mpwr_en",
        .description = "enable/disable VM efuse driver",
        .help = "usage: mpwr_en 1|0\r\n",
        .exec = mpwr_en_callback,
    },
    {
        .name = "ocp",
        .description = "sets ocp threshold (%)",
        .help = "usage: ocp < 0 to 100 >\r\n",
        .exec = ocp_exec_callback,
    },
    {
        .name = "foc",
        .description = "set foc parameters",
        .help = "usage: foc [print|vq[mv]|vd[mv]|vbus[mv]|count_rate]\r\n",
        .exec = foc_cmd_cb,
    }};

// root directory handler
static struct ush_node_object root;
// cmd commands handler
static struct ush_node_object cmd;

void shell_init(void) {

  // initialize microshell instance
  ush_init(&ush, &ush_desc);

  // add custom commands
  ush_commands_add(&ush, &cmd, cmd_files,
                   sizeof(cmd_files) / sizeof(cmd_files[0]));

  // mount root directory (root must be first)
  ush_node_mount(&ush, "/", &root, root_files,
                 sizeof(root_files) / sizeof(root_files[0]));
}

inline void shell_update(void) {
  // service microshell instance
  ush_service(&ush);
}

char *timestamp(void) {
  static char timestamp_msg[32];
  uint64_t usec = timer_us_get();
  snprintf(timestamp_msg, sizeof(timestamp_msg), "[%15lu\t] ", (uint32_t)usec);
  return timestamp_msg;
}
