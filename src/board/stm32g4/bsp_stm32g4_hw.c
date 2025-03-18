
#include "bsp.h"
#include "hal.h"
#include "hal_stm32_pwm.h"
#include "hal_stm32_qenc.h"
#include "log.h"

//------------------------------------------------------+
// PWM Config
//------------------------------------------------------+

void board_pwm_setup(void) {

  board_t *brd = board_get_handle();

  hal_pwm_t pwma = {.regs = HRTIM1, .channel = PWM_HRTIM_TIM_A};
  hal_pwm_t pwmb = {.regs = HRTIM1, .channel = PWM_HRTIM_TIM_F};
  hal_pwm_t pwmc = {.regs = HRTIM1, .channel = PWM_HRTIM_TIM_E};

  brd->hw.mcpwm.pwm[0] = pwma;
  brd->hw.mcpwm.pwm[1] = pwmb;
  brd->hw.mcpwm.pwm[2] = pwmc;

  printf("%s", timestamp());
  if (hal_pwm_3ph_init(&brd->hw.mcpwm, 50000, 100) != 0) {
    LOG_FAIL("PWM");
  } else {
    LOG_OK("PWM");
  }

  float duty_cycles[3] = {0.5f, 0.5f, 0.5f};
  hal_pwm_3ph_set_duty_f32(&brd->hw.mcpwm, duty_cycles);
  hal_stm32_pwm_enable_adc_trigger(&brd->hw.mcpwm.pwm[0]);
  hal_stm32_pwm_enable_period_interrupt(&brd->hw.mcpwm.pwm[0]);
  hal_pwm_3ph_start(&brd->hw.mcpwm);
}

//------------------------------------------------------
// SPI Config
//------------------------------------------------------

__attribute__((unused)) static void board_spi_setup(void) {
  // brd.spi3 =
  //     (struct spi){

  //         .instance = SPI3,
  //         .data_size = 8,
  //         .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
  //         .polarity = 0,
  //         .phase = 1,
  //     },
  //
  // brd.spi4 =
  //     (struct spi){
  //         .instance = SPI4,
  //         .data_size = 16,
  //         .baudrate = SPI_BAUDRATE_PCLK_DIV_128,
  //         .polarity = 0,
  //         .phase = 1,
  //     },
  //
  // gpio_pin_set(&brd.io.spi3_menc1_cs);
  // spi_init_master(&brd.spi3);
  //
  // gpio_pin_set(&brd.io.spi4_gd_cs);
  // gpio_pin_set(&brd.io.spi4_ltc_cs);
  // spi_init_master(&brd.spi4);
}

//------------------------------------------------------
// Gate Driver Config DRV8353
//------------------------------------------------------

// static uint8_t drv835x_spi_transfer(uint16_t data_tx, uint16_t *data_rx) {

//   gpio_pin_clear(&brd.io.spi4_gd_cs);
//   spi_transfer(&brd.spi4, data_tx, data_rx);
//   gpio_pin_set(&brd.io.spi4_gd_cs);

//   return 0;
// }

// static uint8_t drv835x_drv_en(uint8_t state) {
//   if (state) {
//     gpio_pin_set(&brd.io.drive_enable);
//   } else {
//     gpio_pin_clear(&brd.io.drive_enable);
//   }
//   return 0;
// }

// struct drv835x_interface drv835x_io = {
//   .drive_enable = drv835x_drv_en,
//   .spi_transfer = drv835x_spi_transfer,
// };

__attribute__((unused)) static void board_gate_driver_setup(void) {
  // brd.gate_driver.io = &drv835x_io;
}

//------------------------------------------------------
// Encoder
//------------------------------------------------------
static qenc_t menc_abz = (qenc_t){
    .timer = TIM2,
    .mode = QENC_MODE_ABZ,
    .cpr = 2500,
};

uint32_t menc_read(void) { return qenc_read(&menc_abz); }
uint32_t menc_get_cpr(void) { return menc_abz.cpr; }
void menc_set_offset(uint32_t offset) { qenc_load(&menc_abz, offset); }
void menc_update(void) { __NOP(); }

hal_encoder_ops_t menc_ops = (hal_encoder_ops_t){
    .read = menc_read,
    .get_cpr = menc_get_cpr,
    .set_offset = menc_set_offset,
    .update = menc_update,
};

void board_encoder_setup(void) {
  board_t *brd = board_get_handle();

  qenc_init(&menc_abz);
  hal_encoder_init(&brd->hw.encoder, menc_ops);
}

//------------------------------------------------------
// NTCs
//------------------------------------------------------

void board_ntc_setup(void) {
  board_t *brd = board_get_handle();
  brd->hw.ntc_phase_a = (sensor_ntc_t){.B = 3380.0f,
                                       .rth_25C = 10e3,
                                       .rs = 10e3,
                                       .rcmc = 0.0f,
                                       .vs = 3.3,
                                       .config = NTC_CONFIG_BOT,
                                       .analog_input = &brd->ai.temp_a};

  brd->hw.ntc_phase_b = (sensor_ntc_t){.B = 3380.0f,
                                       .rth_25C = 10e3,
                                       .rs = 10e3,
                                       .rcmc = 0.0f,
                                       .vs = 3.3,
                                       .config = NTC_CONFIG_BOT,
                                       .analog_input = &brd->ai.temp_b};

  brd->hw.ntc_phase_c = (sensor_ntc_t){.B = 3380.0f,
                                       .rth_25C = 10e3,
                                       .rs = 10e3,
                                       .rcmc = 0.0f,
                                       .vs = 3.3,
                                       .config = NTC_CONFIG_BOT,
                                       .analog_input = &brd->ai.temp_c};

  brd->hw.ntc_motor = (sensor_ntc_t){.B = 3380.0f,
                                     .rth_25C = 10e3,
                                     .rs = 1e3,
                                     .rcmc = 100.0f,
                                     .vs = 3.3,
                                     .config = NTC_CONFIG_BOT,
                                     .analog_input = &brd->ai.temp_m};
}
