/*--------------------------------------------------------------------------
Board Support Package
File   : bsp.h
--------------------------------------------------------------------------*/

#pragma once

#include "hal.h"

//------------------------------------------------------+
// Board Variant
//------------------------------------------------------+
/*#define BSP_G4_NUKLEO*/
#define BSP_MLB_revA
// #define BSP_MLB_G4
// #define BSP_F50_G4

//------------------------------------------------------+
// Shell Interface Selection
//------------------------------------------------------+
#define SHELL_INTERFACE_USB
/*#define SHELL_INTERFACE_UART*/

//------------------------------------------------------
// GPIOs
//------------------------------------------------------
typedef struct board_s {

  struct board_dio_s {
    /*// Indicators*/
    hal_gpio_t led_red;
    hal_gpio_t led_green;
    hal_gpio_t led_blue;

    // Motor Drive Control
    hal_gpio_t mpwr_en;
    hal_gpio_t flt_mwr_n;
    hal_gpio_t flt_drv_n;
    hal_gpio_t motor_en;
    hal_gpio_t sto_a_n;
    hal_gpio_t sto_b_n;
    hal_gpio_t mc_status;
    hal_gpio_t mc_error_n;

    // Motor PWMs
    hal_gpio_t pwm_ah;
    hal_gpio_t pwm_al;
    hal_gpio_t pwm_bh;
    hal_gpio_t pwm_bl;
    hal_gpio_t pwm_ch;
    hal_gpio_t pwm_cl;

    // Misc
    hal_gpio_t tc_pwr_en;
    hal_gpio_t enc_a_pin;
    hal_gpio_t enc_b_pin;
    hal_gpio_t test_pin0;

    // SPI1
    hal_gpio_t spi1_mosi;
    hal_gpio_t spi1_miso;
    hal_gpio_t spi1_clk;
    hal_gpio_t spi1_cs_n;

    // SPI2
    hal_gpio_t spi2_mosi;
    hal_gpio_t spi2_miso;
    hal_gpio_t spi2_clk;
    hal_gpio_t spi2_cs_n;

    // SPI4
    hal_gpio_t spi4_mosi;
    hal_gpio_t spi4_miso;
    hal_gpio_t spi4_clk;
    hal_gpio_t spi4_men_cs_n;
    hal_gpio_t spi4_oen_cs_n;

    // UART Console
    hal_gpio_t console_tx;
    hal_gpio_t console_rx;

    // USB
    hal_gpio_t usb_dp;
    hal_gpio_t usb_dm;

  } dio;

  struct board_ai_s {

    hal_analog_input_t vbatt_mon;
    hal_analog_input_t vgd_mon;
    hal_analog_input_t vl_mon;
    hal_analog_input_t temp_a;
    hal_analog_input_t temp_b;
    hal_analog_input_t temp_c;
    hal_analog_input_t temp_m;

    hal_analog_input_t vm_fb;
    hal_analog_input_t va_fb;
    hal_analog_input_t vb_fb;
    hal_analog_input_t vc_fb;
    hal_analog_input_t im_fb;
    hal_analog_input_t ia_fb;
    hal_analog_input_t ib_fb;
    hal_analog_input_t ic_fb;

  } ai;

  struct board_hw_t {
    hal_pwm_3ph_t mcpwm;
    hal_encoder_t encoder;
  } hw;

  struct board_com_t {
    hal_uart_t console;
  } com;

} board_t;

board_t *board_get_handle(void);
int board_init(void);
void board_hw_setup(void);

int board_load_pinmap(board_t *self);

int board_start_bootloader(board_t *self);
