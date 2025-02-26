
#ifndef __HAL_GPIO_H__
#define __HAL_GPIO_H__

#include <stdint.h>

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
  GPIO_TYPE_PUSH_PULL,
  GPIO_TYPE_OPEN_DRAIN,
} hal_gpio_output_t;

typedef enum {
  GPIO_PULL_NONE,
  GPIO_PULL_UP,
  GPIO_PULL_DOWN,
} hal_gpio_pull_t;

typedef struct {
  void *port;
  uint16_t pin;
  hal_gpio_mode_t mode;
  hal_gpio_speed_t speed;
  hal_gpio_output_t output;
  hal_gpio_pull_t pullup;

} hal_gpio_t;

void hal_gpio_pin_init(const hal_gpio_t *pin);
void hal_gpio_pin_deinit(hal_gpio_t *pin);
inline void hal_gpio_pin_set(hal_gpio_t *pin);
inline void hal_gpio_pin_clear(hal_gpio_t *pin);
inline void hal_gpio_pin_toggle(hal_gpio_t *pin);
inline uint16_t hal_gpio_pin_read(hal_gpio_t *pin);
inline void hal_gpio_pin_write(hal_gpio_t *pin, uint16_t value);

//------------------------------------------------------
// Inline Functions
//------------------------------------------------------

inline void gpio_pin_clear(gpio_t *pin) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
  gpio->BSRR |= (1 << (pin->pin + 16));
}

inline void gpio_pin_set(gpio_t *pin) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
  gpio->BSRR |= (1 << pin->pin);
}

inline uint32_t gpio_pin_get(gpio_t *pin) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
  return (gpio->IDR & (1 << pin->pin)) ? 1 : 0;
}

inline void gpio_pin_toggle(gpio_t *pin) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
  gpio->ODR ^= (1 << pin->pin);
}

inline void gpio_pin_write(gpio_t *pin, uint16_t value) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
  gpio->ODR = value;
}

inline uint16_t gpio_pin_read(gpio_t *pin) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)(GPIOA_BASE + (0x400 * pin->port));
  return gpio->IDR;
}

#endif // STM32G4_GPIO_H
