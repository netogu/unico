
#include "bsp.h"
#include "hal_encoder.h"
#include "log.h"
#include "stm32g4_qenc.h"

//------------------------------------------------------+
// PWM Config
//------------------------------------------------------+

void board_pwm_setup(void) {

  board_t *brd = board_get_handle();

  brd->hw.mcpwm =

      (pwm_3ph_t){
          .pwma =
              (pwm_t){
                  .options =
                      {
                          .pwm_timer = PWM_TIMER_HRTIM1,
                          .pwm_channel = PWM_HRTIM_TIM_A,
                          .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
                          .polarity = HRTIM_PWM_POLARITY_NORMAL,
                      },
              },

          .pwmb =
              (pwm_t){
                  .options =
                      {
                          .pwm_timer = PWM_TIMER_HRTIM1,
                          .pwm_channel = PWM_HRTIM_TIM_F,
                          .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
                          .polarity = HRTIM_PWM_POLARITY_NORMAL,
                      },
              },

          .pwmc =
              (pwm_t){
                  .options =
                      {
                          .pwm_timer = PWM_TIMER_HRTIM1,
                          .pwm_channel = PWM_HRTIM_TIM_E,
                          .output_mode = HRTIM_PWM_OUTPUT_COMPLEMENTARY,
                          .polarity = HRTIM_PWM_POLARITY_NORMAL,
                      },
              },

          .mode = PWM_3PHASE_MODE_6PWM,
      },

  printf(timestamp());
  if (pwm_3ph_init(&brd->hw.mcpwm, 50000, 200) != 0) {
    LOG_FAIL("PWM");
  } else {
    LOG_OK("PWM");
  }

  pwm_3ph_set_duty(&brd->hw.mcpwm, 0.5f, 0.5f, 0.5f);
  pwm_enable_adc_trigger(&brd->hw.mcpwm.pwma);
  pwm_enable_period_interrupt(&brd->hw.mcpwm.pwma);
  pwm_3ph_start(&brd->hw.mcpwm);
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
    .cpr = 400,
};

uint32_t menc_read(void) { return qenc_read(&menc_abz); }
uint32_t menc_get_cpr(void) { return menc_abz.cpr; }
void menc_set_offset(uint32_t offset) { qenc_load(&menc_abz, offset); }
void menc_update(void) { __NOP(); }

encoder_ops_t menc_ops = (encoder_ops_t){
    .read = menc_read,
    .get_cpr = menc_get_cpr,
    .set_offset = menc_set_offset,
    .update = menc_update,
};

void board_encoder_setup(void) {
  board_t *brd = board_get_handle();

  qenc_init(&menc_abz);
  encoder_init(&brd->hw.encoder, menc_ops);
}
