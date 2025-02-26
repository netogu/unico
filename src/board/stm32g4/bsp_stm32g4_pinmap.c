#include "bsp.h"

int board_load_pinmap(board_t *brd) {

  brd->dio = (struct board_dio_s){
      .led_red =
          (hal_gpio_t){
              .port = GPIOB,
              .pin = 2,
              .mode = GPIO_MODE_OUTPUT,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = 0,
          },

#ifdef BSP_G4_NUKLEO
      .led_green =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 5,
              .mode = GPIO_MODE_OUTPUT,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF0,
          },
#else
      .led_green =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 12,
              .mode = GPIO_MODE_OUTPUT,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = 0,
          },
#endif

      .led_blue =
          (hal_gpio_t){
              .port = GPIOF,
              .pin = 9,
              .mode = GPIO_MODE_OUTPUT,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = 0,
          },

      .mpwr_en =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 13,
              .mode = GPIO_MODE_OUTPUT,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_LOW,
              .af = 0,
          },

      .flt_mwr_n =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 14,
              .mode = GPIO_MODE_INPUT,
              .output = GPIO_OUTPUT_OPEN_DRAIN,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_LOW,
              .af = 0,
          },

      .test_pin0 = // on MC_STATUS
      (hal_gpio_t){
          .port = GPIOD,
          .pin = 15,
          .mode = GPIO_MODE_OUTPUT,
          .output = GPIO_OUTPUT_PUSH_PULL,
          .pull = GPIO_PULL_UP,
          .speed = GPIO_SPEED_HIGH,
          .af = 0,
      },

      .usb_dp =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 12,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_VERY_HIGH,
              .af = 0,
          },

      .usb_dm =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 11,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_VERY_HIGH,
              .af = 0,
          },
#ifdef BSP_G4_NUKLEO
      // on LPUART1
      .console_tx =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 2,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_LOW,
              .af = 12,
          },
      .console_rx =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 3,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_LOW,
              .af = 12,
          },
#elif defined(BSP_MLB_revA)
      // on USART1
      .console_tx =
          (hal_gpio_t){
              .port = GPIOE,
              .pin = 0,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_MEDIUM,
              .af = 7,
          },
      .console_rx =
          (hal_gpio_t){
              .port = GPIOE,
              .pin = 1,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_MEDIUM,
              .af = 7,
          },
#endif

      // PWMs
      .pwm_ah =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 8,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = 13,
          },

      .pwm_al =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 9,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = 13,
          },

      .pwm_bh =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 6,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = 13,
          },
      .pwm_bl =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 7,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = 13,
          },

      .pwm_ch =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 8,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = 3,
          },

      .pwm_cl =
          (hal_gpio_t){
              .port = GPIOC,
              .pin = 9,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = 3,
          },

#if defined(BSP_MLB_revA)

      .enc_a_pin =
          (hal_gpio_t){
              .port = GPIOD,
              .pin = 3,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .speed = GPIO_SPEED_HIGH,
              .af = 2,
          },

      .enc_b_pin =
          (hal_gpio_t){
              .port = GPIOD,
              .pin = 4,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .speed = GPIO_SPEED_HIGH,
              .af = 2,
          },

#elif defined(BSP_G4_NUKLEO)

      .enc_a_pin =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 0,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .speed = GPIO_SPEED_HIGH,
              .af = 2,
          },

      .enc_b_pin =
          (hal_gpio_t){
              .port = GPIOA,
              .pin = 1,
              .mode = GPIO_MODE_ALTERNATE,
              .output = GPIO_OUTPUT_PUSH_PULL,
              .speed = GPIO_SPEED_HIGH,
              .af = 2,
          },
#endif

  };

  return 0;
}
