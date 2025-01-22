/*--------------------------------------------------------------------------
Board Support Package
File   : bsp.h
--------------------------------------------------------------------------*/

#pragma once

#include "encoder.h"
#include "stm32g4.h"

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
    gpio_t led_red;
    gpio_t led_green;
    gpio_t led_blue;

    // Motor Drive Control
    gpio_t mpwr_en;
    gpio_t flt_mwr_n;
    gpio_t flt_drv_n;
    gpio_t motor_en;
    gpio_t sto_a_n;
    gpio_t sto_b_n;
    gpio_t mc_status;
    gpio_t mc_error_n;

    // Motor PWMs
    gpio_t pwm_ah;
    gpio_t pwm_al;
    gpio_t pwm_bh;
    gpio_t pwm_bl;
    gpio_t pwm_ch;
    gpio_t pwm_cl;

    // Misc
    gpio_t tc_pwr_en;
    gpio_t enc_a_pin;
    gpio_t enc_b_pin;
    gpio_t test_pin0;

    // SPI1
    gpio_t spi1_mosi;
    gpio_t spi1_miso;
    gpio_t spi1_clk;
    gpio_t spi1_cs_n;

    // SPI2
    gpio_t spi2_mosi;
    gpio_t spi2_miso;
    gpio_t spi2_clk;
    gpio_t spi2_cs_n;

    // SPI4
    gpio_t spi4_mosi;
    gpio_t spi4_miso;
    gpio_t spi4_clk;
    gpio_t spi4_men_cs_n;
    gpio_t spi4_oen_cs_n;

    // UART Console
    gpio_t console_tx;
    gpio_t console_rx;

    // USB
    gpio_t usb_dp;
    gpio_t usb_dm;

  } dio;

  struct board_ai_s {

    adc_input_t vbatt_mon;
    adc_input_t vgd_mon;
    adc_input_t temp_a;
    adc_input_t temp_b;
    adc_input_t temp_c;
    adc_input_t temp_m;

    adc_input_t vm_fb;
    adc_input_t va_fb;
    adc_input_t vb_fb;
    adc_input_t vc_fb;
    adc_input_t im_fb;
    adc_input_t ia_fb;
    adc_input_t ib_fb;
    adc_input_t ic_fb;

  } ai;

  struct board_hw_t {
    pwm_3ph_t mcpwm;
    encoder_t encoder;
  } hw;

  struct board_com_t {
    uart_t console;
  } com;

} board_t;

board_t *board_get_handle(void);
int board_init(void);
void board_hw_setup(void);

int board_load_pinmap(board_t *self);

int board_start_bootloader(board_t *self);
