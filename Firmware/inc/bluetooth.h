#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#define F_CPU 16000000UL      // 16 MHz Nano clock
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

#ifdef __cplusplus
extern "C" {
#endif

// Function Prototypes
void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
char uart_getc(void);

#ifdef __cplusplus
}
#endif

#endif
