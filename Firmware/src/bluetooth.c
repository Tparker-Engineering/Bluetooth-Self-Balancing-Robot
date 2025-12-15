#include "bluetooth.h"

void uart_init(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE);

    // Enable transmitter and receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    // 8-bit data, 1 stop, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));   // Wait until TX buffer empty
    UDR0 = c;
}

void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

char uart_getc(void)
{
    while (!(UCSR0A & (1 << RXC0)));    // Wait for data
    return UDR0;
}
