#include "bsp.h"

int board_load_pinmap(board_t *brd) {

  brd->dio = (struct board_dio_s){
      .led_red =
          (gpio_t){
              .port = GPIO_PORT_B,
              .pin = GPIO_PIN_2,
              .mode = GPIO_MODE_OUTPUT,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF0,
          },

#ifdef BSP_G4_NUKLEO
      .led_green =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_5,
              .mode = GPIO_MODE_OUTPUT,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF0,
          },
#else
      .led_green =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_12,
              .mode = GPIO_MODE_OUTPUT,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF0,
          },
#endif

      .led_blue =
          (gpio_t){
              .port = GPIO_PORT_F,
              .pin = GPIO_PIN_9,
              .mode = GPIO_MODE_OUTPUT,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF0,
          },

      .mpwr_en =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_13,
              .mode = GPIO_MODE_OUTPUT,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_LOW,
              .af = GPIO_AF0,
          },

      .flt_mwr_n =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_14,
              .mode = GPIO_MODE_INPUT,
              .type = GPIO_TYPE_OPEN_DRAIN,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_LOW,
              .af = GPIO_AF0,
          },

      .test_pin0 =
          (gpio_t){
              .port = GPIO_PORT_E,
              .pin = GPIO_PIN_3,
              .mode = GPIO_MODE_OUTPUT,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_UP,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF0,
          },

      .usb_dp =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_12,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_VERY_HIGH,
              .af = GPIO_AF0,
          },

      .usb_dm =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_11,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_VERY_HIGH,
              .af = GPIO_AF0,
          },
#ifdef BSP_G4_NUKLEO
      // on LPUART1
      .console_tx =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_2,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_LOW,
              .af = GPIO_AF12,
          },
      .console_rx =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_3,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_LOW,
              .af = GPIO_AF12,
          },
#elif defined(BSP_MLB_revA)
      // on USART1
      .console_tx =
          (gpio_t){
              .port = GPIO_PORT_E,
              .pin = GPIO_PIN_0,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_MEDIUM,
              .af = GPIO_AF7,
          },
      .console_rx =
          (gpio_t){
              .port = GPIO_PORT_E,
              .pin = GPIO_PIN_1,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_MEDIUM,
              .af = GPIO_AF7,
          },
#endif

      // PWMs
      .pwm_ah =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_8,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF13,
          },

      .pwm_al =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_9,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF13,
          },

      .pwm_bh =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_6,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF13,
          },
      .pwm_bl =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_7,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF13,
          },

      .pwm_ch =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_8,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF3,
          },

      .pwm_cl =
          (gpio_t){
              .port = GPIO_PORT_C,
              .pin = GPIO_PIN_9,
              .mode = GPIO_MODE_ALTERNATE,
              .type = GPIO_TYPE_PUSH_PULL,
              .pull = GPIO_PULL_NONE,
              .speed = GPIO_SPEED_HIGH,
              .af = GPIO_AF3,
          },

      .enc_a_pin =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_0,
              .mode = GPIO_MODE_ALTERNATE,
              .af = GPIO_AF2,
              .type = GPIO_TYPE_PUSH_PULL,
              .speed = GPIO_SPEED_HIGH,
          },

      .enc_b_pin =
          (gpio_t){
              .port = GPIO_PORT_A,
              .pin = GPIO_PIN_1,
              .mode = GPIO_MODE_ALTERNATE,
              .af = GPIO_AF2,
              .type = GPIO_TYPE_PUSH_PULL,
              .speed = GPIO_SPEED_HIGH,
          },

  };

  return 0;
}
