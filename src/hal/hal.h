#ifndef __HAL_H
#define __HAL_H

#include "rtos.h"
#include "stdbool.h"

#define NOCHAR '\0'

/*--------------------------------------------------*/
//  Analog
/*--------------------------------------------------*/

typedef struct {
  char *name;
  uint8_t channel;
  float scale;
  float offset;
  char *units;
  volatile uint32_t *data;
} hal_analog_input_t;

uint32_t hal_analog_read_raw(hal_analog_input_t *self);
float hal_analog_read_f32(hal_analog_input_t *self);

/*--------------------------------------------------*/
//  Encoders
/*--------------------------------------------------*/

#include <stdint.h>

typedef struct {
  uint32_t (*read)(void);
  void (*set_offset)(uint32_t offset);
  uint32_t (*get_cpr)(void);
  void (*update)(void);

} hal_encoder_ops_t;

typedef struct {
  hal_encoder_ops_t ops;
  uint32_t cpr;

} hal_encoder_t;

int hal_encoder_init(hal_encoder_t *self, hal_encoder_ops_t ops);
int hal_encoder_update(hal_encoder_t *self);
int32_t hal_encoder_read_count(hal_encoder_t *self);
void hal_encoder_set_offset(hal_encoder_t *self, uint32_t count);
int32_t hal_encoder_read_angle_q31(hal_encoder_t *self);
float hal_encoder_read_angle_f32(hal_encoder_t *self);

/*--------------------------------------------------*/
//  UART
/*--------------------------------------------------*/

#define UART_BUFFER_SIZE 1024

typedef struct {
  uint8_t buffer[UART_BUFFER_SIZE];
  uint8_t head;
  uint8_t tail;
  uint32_t size;
} hal_uart_fifo_t;

typedef struct {
  void *port;
  hal_uart_fifo_t rx_fifo;
  hal_uart_fifo_t tx_fifo;
  uint16_t tx_dma_current_transfer_size;
} hal_uart_t;

typedef struct {
  uint32_t baudrate;
  enum uart_word_len_e {
    UART_DATA_BITS_8 = 0,
    UART_DATA_BITS_9 = 1,
    UART_DATA_BITS_7 = 2,
  } word_length;
  enum uart_stop_bits_e {
    UART_STOP_BITS_1 = 0,
    UART_STOP_BITS_2 = 2,
  } stop_bits;
  enum uart_parity_e {
    UART_PARITY_EVEN = 0,
    UART_PARITY_ODD = 1,
    UART_PARITY_NONE = 2,
  } parity;
  enum uart_mode_e {
    UART_MODE_RX = 0,
    UART_MODE_TX = 1,
    UART_MODE_RX_TX = 2,
  } mode;
  enum uart_flow_control_e {
    UART_FLOW_CONTROL_NONE = 0,
    UART_FLOW_CONTROL_RTS = 1,
    UART_FLOW_CONTROL_CTS = 2,
    UART_FLOW_CONTROL_RTS_CTS = 3,
  } flow_control;
} hal_uart_config_t;

int hal_uart_init(hal_uart_t *self, const hal_uart_config_t *config);
int hal_uart_init_dma(hal_uart_t *self, const hal_uart_config_t *config);
int hal_uart_write(hal_uart_t *self, uint8_t *data, uint16_t size);
int hal_uart_read(hal_uart_t *self, uint8_t *data, uint16_t size);
int hal_uart_start_dma_tx_transfer(hal_uart_t *self);
void hal_uart_service_rx_dma(hal_uart_t *self);
int hal_uart_fifo_push(hal_uart_fifo_t *self, uint8_t byte);
int hal_uart_fifo_pop(hal_uart_fifo_t *self, uint8_t *byte);
uint16_t hal_uart_fifo_get_linear_size(hal_uart_fifo_t *self);

/*--------------------------------------------------*/
//  CLI
/*--------------------------------------------------*/

#define USB_STR_SERIALNO_LEN 8

extern SemaphoreHandle_t usb_mutex;

void cli_usb_init(void);
int cli_usb_putc(char tx_char);
char cli_usb_getc(void);

extern SemaphoreHandle_t uart_mutex;

int cli_uart_init(hal_uart_t *port);
int cli_uart_putc(char tx_char);
char cli_uart_getc(void);
int cli_uart_puts(const char *str);
int cli_printf(const char *format, ...);
uint32_t cli_uart_tx_pending(hal_uart_t *port);

/*--------------------------------------------------*/
//  Timer
/*--------------------------------------------------*/

typedef struct hal_timer_s hal_timer_t;

hal_timer_t *timer_create(uint32_t period_us, void (*on_timeout_cb)(void));
void timer_start(hal_timer_t *self);
void timer_stop(hal_timer_t *self);
//
// FreeRTOS Stats Timer
void hal_timer_us_init(void);
uint64_t hal_timer_us_get(void);

/*--------------------------------------------------*/
//  GPIO
/*--------------------------------------------------*/

typedef enum {
  GPIO_MODE_INPUT,
  GPIO_MODE_OUTPUT,
  GPIO_MODE_ALTERNATE,
  GPIO_MODE_ANALOG,
} hal_gpio_mode_t;

typedef enum {
  GPIO_SPEED_LOW,
  GPIO_SPEED_MEDIUM,
  GPIO_SPEED_HIGH,
  GPIO_SPEED_VERY_HIGH,
} hal_gpio_speed_t;

typedef enum {
  GPIO_OUTPUT_PUSH_PULL,
  GPIO_OUTPUT_OPEN_DRAIN,
} hal_gpio_output_t;

typedef enum {
  GPIO_PULL_NONE,
  GPIO_PULL_UP,
  GPIO_PULL_DOWN,
} hal_gpio_pull_t;

typedef struct {
  void *port;
  uint16_t pin;
  uint16_t af;
  hal_gpio_mode_t mode;
  hal_gpio_speed_t speed;
  hal_gpio_output_t output;
  hal_gpio_pull_t pull;

} hal_gpio_t;

void hal_gpio_init(const hal_gpio_t *self);
void hal_gpio_deinit(hal_gpio_t *self);
void hal_gpio_set(hal_gpio_t *self);
void hal_gpio_clear(hal_gpio_t *self);
void hal_gpio_toggle(hal_gpio_t *self);
uint16_t hal_gpio_read(hal_gpio_t *self);
void hal_gpio_write(hal_gpio_t *self, bool state);

/*--------------------------------------------------*/
//  PWM
/*--------------------------------------------------*/

typedef struct {
  void *regs;
  uint32_t channel;
  void *options;
} hal_pwm_t;

typedef struct {
  hal_pwm_t pwm[3];
  void *options;
} hal_pwm_3ph_t;

int hal_pwm_init(hal_pwm_t *self, uint32_t freq_hz, uint32_t dt_ns);
int hal_pwm_deinit(hal_pwm_t *self);
int hal_pwm_start(hal_pwm_t *self);
int hal_pwm_stop(hal_pwm_t *self);
int hal_pwm_set_frequency(hal_pwm_t *self, uint32_t freq_hz);
int hal_pwm_set_duty_f32(hal_pwm_t *self, float duty_u);
int hal_pwm_set_duty_q31(hal_pwm_t *self, int32_t duty_u);
int hal_pwm_set_deadtime(hal_pwm_t *self, uint32_t dt_ns);

int hal_pwm_3ph_init(hal_pwm_3ph_t *self, uint32_t freq_hz, uint32_t dt_ns);
int hal_pwm_3ph_start(hal_pwm_3ph_t *self);
int hal_pwm_3ph_stop(hal_pwm_3ph_t *self);
int hal_pwm_3ph_set_frequency(hal_pwm_3ph_t *self, uint32_t freq_hz);
int hal_pwm_3ph_set_duty_f32(hal_pwm_3ph_t *self, float duty_abc_u[]);

/*--------------------------------------------------*/
//  System
/*--------------------------------------------------*/

typedef struct system_id_s {
  uint32_t device_id;
  uint16_t flash_size_kb;
  uint16_t package_type;
  uint32_t uid[3];
} system_id_t;

void system_get_id(system_id_t *id);

#endif // __HAL_H
