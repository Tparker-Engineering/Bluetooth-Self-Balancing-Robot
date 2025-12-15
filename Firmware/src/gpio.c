#include "gpio.h"
#include <avr/interrupt.h>

// ========================================
// pin mapping for arduino nano
// ========================================
//
// Digital 0-7  -> PORTD bits 0–7
// Digital 8-13 -> PORTB bits 0–5
// Analog A0–A5 -> PORTC bits 0–5 (14–19)
//
#define NOT_A_PIN 0xFF
enum { porta=1, portb=2, portc=3, portd=4 };

// Helper functions
static inline volatile uint8_t* pinToPort(uint8_t pin)
{
    if (pin <= 7)           // D0–D7 : PORTD
        return portd;
    else if (pin <= 13)     // D8–D13 : PORTB
        return portb;
    else if (pin <= 19)     // A0–A5 : PORTC
        return portc;
    else
        return NOT_A_PIN;           // invalid pin
}


static inline uint8_t pinToMask(uint8_t pin)
{
    if (pin <= 7)           // D0–D7 : PD0–PD7
        return (1 << pin);
    else if (pin <= 13)     // D8–D13 : PB0–PB5
        return (1 << (pin - 8));
    else if (pin <= 19)     // A0–A5 : PC0–PC5
        return (1 << (pin - 14));
    else
        return 0;
}

static inline volatile uint8_t* portDDR(uint8_t port)
{
    switch (port)
    {
        case portb: return &DDRB;
        case portc: return &DDRC;
        case portd: return &DDRD;
        default: return 0;
    }
}

static inline volatile uint8_t* portOUT(uint8_t port)
{
    switch (port)
    {
        case portb: return &PORTB;
        case portc: return &PORTC;
        case portd: return &PORTD;
        default: return 0;
    }
}

static inline volatile uint8_t* portIN(uint8_t port)
{
    switch (port)
    {
        case portb: return &PINB;
        case portc: return &PINC;
        case portd: return &PIND;
        default: return 0;
    }
}

// =====================
// Function Definitions
// =====================

// Set pin as input, output, or input with pull-up
void gpio_pinMode(uint8_t pin, uint8_t mode)
{
    uint8_t port = pinToPort(pin);
    if (port == NOT_A_PIN) return;
    uint8_t mask = pinToMask(pin);

    volatile uint8_t *ddr = portDDR(port);
    volatile uint8_t *out = portOUT(port);

    uint8_t s = SREG; cli();

    if (mode == GPIO_OUTPUT)
    {
        *ddr |= mask;          // Set as output
    }
    else
    {
        *ddr &= ~mask;         // Input
        if (mode == GPIO_INPUT_PU)
            *out |= mask;      // Enable pull-up
        else
            *out &= ~mask;     // Floating input
    }

    SREG = s;

}

// Write high or low
void gpio_write(uint8_t pin, uint8_t val)
{
    uint8_t port = pinToPort(pin);
    if (port == NOT_A_PIN) return;
    uint8_t mask = pinToMask(pin);

    volatile uint8_t *out = portOUT(port);

    uint8_t s = SREG; cli();
    if (val)
        *out |= mask;
    else
        *out &= ~mask;
    SREG = s;
}

// Read pin value (0 or 1)
uint8_t gpio_read(uint8_t pin)
{
    uint8_t port = pinToPort(pin);
    if (port == NOT_A_PIN) return 0;
    uint8_t mask = pinToMask(pin);

    volatile uint8_t *in = portIN(port);
    return ((*in & mask) ? 1 : 0);
}

// Toggle pin state
void gpio_toggle(uint8_t pin)
{
    uint8_t port = pinToPort(pin);
    if (port == NOT_A_PIN) return;
    uint8_t mask = pinToMask(pin);

    volatile uint8_t *out = portOUT(port);

    uint8_t s = SREG; cli();
    *out ^= mask; //toggle
    SREG = s;
}
