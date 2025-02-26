#include "hal.h"
#include <stdbool.h>

void hal_gpio_init(const hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;

  uint32_t port_ahb2_index = (uint32_t)self->port - (uint32_t)GPIOA;

  // Enable GPIO clock
  RCC->AHB2ENR |= (1 << port_ahb2_index);

  // Set GPIO mode
  gpio->MODER &= ~(0x3 << (self->pin * 2));
  gpio->MODER |= (self->mode << (self->pin * 2));

  // Set GPIO speed
  gpio->OSPEEDR &= ~(0x3 << (self->pin * 2));
  gpio->OSPEEDR |= (self->speed << (self->pin * 2));

  // Set GPIO output type
  gpio->OTYPER &= ~(0x1 << self->pin);
  gpio->OTYPER |= (self->output << self->pin);

  // Clear output if it is an output
  if (self->mode == GPIO_MODE_OUTPUT) {
    gpio->ODR &= ~(1 << self->pin);
  }

  // Set GPIO pull-up/pull-down
  gpio->PUPDR &= ~(0x3 << (self->pin * 2));
  gpio->PUPDR |= (self->pull << (self->pin * 2));

  // Set GPIO alternate function
  gpio->AFR[self->pin / 8] &= ~(0xF << ((self->pin % 8) * 4));
  gpio->AFR[self->pin / 8] |= (self->af << ((self->pin % 8) * 4));
}

void hal_gpio_deinit(hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  gpio->MODER &= ~(0x3 << (self->pin * 2));
  gpio->OSPEEDR &= ~(0x3 << (self->pin * 2));
  gpio->OTYPER &= ~(0x1 << self->pin);
  gpio->PUPDR &= ~(0x3 << (self->pin * 2));
  gpio->AFR[self->pin / 8] &= ~(0xF << ((self->pin % 8) * 4));
}

//------------------------------------------------------
// Inline Functions
//------------------------------------------------------

inline void hal_gpio_clear(hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  gpio->BSRR |= (1 << (self->pin + 16));
}

inline void hal_gpio_set(hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  gpio->BSRR |= (1 << self->pin);
}

inline uint32_t hal_gpio_get(hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  return (gpio->IDR & (1 << self->pin)) ? 1 : 0;
}

inline void hal_gpio_toggle(hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  gpio->ODR ^= (1 << self->pin);
}

inline void hal_gpio_write(hal_gpio_t *self, bool value) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  gpio->ODR = value;
}

inline uint16_t hal_gpio_read(hal_gpio_t *self) {

  GPIO_TypeDef *gpio = (GPIO_TypeDef *)self->port;
  return gpio->IDR;
}
