#ifndef GPIO_H
#define GPIO_H

#include <avr/io.h>
#include <stdint.h>

// =====================
// modes and logic
// =====================
#define GPIO_LOW        0
#define GPIO_HIGH       1

#define GPIO_INPUT      0
#define GPIO_OUTPUT     1
#define GPIO_INPUT_PU   2   // Input with internal pull-up

// =====================
// function, more tba
// =====================
#ifdef __cplusplus
extern "C" {
#endif

void gpio_pinMode(uint8_t pin, uint8_t mode);
void gpio_write(uint8_t pin, uint8_t val);
uint8_t gpio_read(uint8_t pin);
void gpio_toggle(uint8_t pin);
#ifdef __cplusplus
}
#endif

#endif
