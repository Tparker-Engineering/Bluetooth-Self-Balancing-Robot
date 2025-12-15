#include <avr/io.h>
#include <avr/interrupt.h>
#include "motor.h"
#include "gpio.h" 

static Motor *leftMotorPtr  = 0;   // expects encoder on D2 (INT0 / PD2)
static Motor *rightMotorPtr = 0;   // expects encoder on D4 (PCINT20 / PD4)

//PWM support
static void pwm_init_for_pin(uint8_t pin)
{
    switch (pin)
    {
        //Timer0 Fast PWM, non-inverting, clk/64 (976 Hz on 16 MHz)
        case 5: // D5 = PD5 = OC0B
            // Fast PWM: WGM01=1, WGM00=1; OC0B non-inverting (COM0B1=1)
            TCCR0A |= (1<<WGM01) | (1<<WGM00) | (1<<COM0B1);
            TCCR0B |= (1<<CS01) | (1<<CS00);  // prescaler /64
            break;
        case 6: // D6 = PD6 = OC0A
            TCCR0A |= (1<<WGM01) | (1<<WGM00) | (1<<COM0A1);
            TCCR0B |= (1<<CS01) | (1<<CS00);
            break;

        //Timer1 8-bit Fast PWM (WGM10=1), non-inverting, clk/64
        case 9: // D9 = PB1 = OC1A
            TCCR1A |= (1<<WGM10) | (1<<COM1A1);
            TCCR1B |= (1<<CS11) | (1<<CS10);  // /64
            break;
        case 10: // D10 = PB2 = OC1B
            TCCR1A |= (1<<WGM10) | (1<<COM1B1);
            TCCR1B |= (1<<CS11) | (1<<CS10);
            break;

        // Timer2 Fast PWM, non-inverting, clk/64 (976 Hz)
        case 3: // D3 = PD3 = OC2B
            TCCR2A |= (1<<WGM21) | (1<<WGM20) | (1<<COM2B1);
            TCCR2B |= (1<<CS22); // prescaler /64
            break;
        case 11: // D11 = PB3 = OC2A
            TCCR2A |= (1<<WGM21) | (1<<WGM20) | (1<<COM2A1);
            TCCR2B |= (1<<CS22);
            break;

        default:
            // non-PWM pin: nothing to init (will get “0 or 255 only” if used)
            break;
    }
}

static void pwm_set_duty(uint8_t pin, uint8_t duty)
{
    switch (pin)
    {
        case 5:  OCR0B = duty; break;  // D5
        case 6:  OCR0A = duty; break;  // D6
        case 9:  OCR1A = duty; break;  // D9 (8-bit when WGM10=1)
        case 10: OCR1B = duty; break;  // D10
        case 3:  OCR2B = duty; break;  // D3
        case 11: OCR2A = duty; break;  // D11
        default:
            // If pin is not hardware-PWM, emulate: 0->LOW, >0->HIGH 
            if (duty == 0)      gpio_write(pin, 0);
            else if (duty >= 255) gpio_write(pin, 1);
            else                gpio_write(pin, 1); // on/off
            break;
    }
}

//Encoder ISRs
// Left encoder on D2 (PD2 / INT0), rising edge
ISR(INT0_vect)  // Left encoder, D2
{
    static uint8_t last = 0;
    uint8_t current = (PIND & (1<<PIND2)) ? 1 : 0;

    if (current != last)   // <-- count on any change, like vendor's CHANGE
    {
        if (leftMotorPtr)
        {
            uint8_t dir = gpio_read(leftMotorPtr->pinDir);
            if (dir)
                leftMotorPtr->count--;   // reverse
            else
                leftMotorPtr->count++;   // forward
        }
    }
    last = current;
}

ISR(PCINT2_vect)  // Right encoder, D4
{
    static uint8_t last = 0;
    uint8_t current = (PIND & (1<<PIND4)) ? 1 : 0;

    if (current != last)   // <-- any edge
    {
        if (rightMotorPtr)
        {
            uint8_t dir = gpio_read(rightMotorPtr->pinDir);
            if (dir)
                rightMotorPtr->count--;
            else
                rightMotorPtr->count++;
        }
    }
    last = current;
}

//Public API
void motorInit(Motor *m, uint8_t dir, uint8_t pwm, uint8_t stby, uint8_t enc)
{
    m->pinDir  = dir;
    m->pinPWM  = pwm;
    m->pinStby = stby;
    m->pinEnc  = enc;
    m->count = 0;
    m->prevCount = 0;
    m->speed = 0;

    // Configure pins via your GPIO driver
    gpio_pinMode(m->pinDir,  GPIO_OUTPUT);
    gpio_pinMode(m->pinPWM,  GPIO_OUTPUT);
    gpio_pinMode(m->pinStby, GPIO_OUTPUT);
    gpio_pinMode(m->pinEnc,  GPIO_INPUT_PU);  // pull-up 

    // Enable driver (standby = HIGH)
    gpio_write(m->pinStby, 1);

    // Initialize hardware PWM for selected pin
    pwm_init_for_pin(m->pinPWM);
    pwm_set_duty(m->pinPWM, 0);

    //Encoder routing
    if (enc == 2) // D2 -> INT0 rising edge
    {
        leftMotorPtr = m;

        // External Interrupt 0 on rising edge: ISC01=1, ISC00=1
        EICRA |= (1<<ISC01) | (1<<ISC00);
        EIFR  |= (1<<INTF0);     // clear any pending
        EIMSK |= (1<<INT0);      // enable INT0
    }
    else if (enc == 4) // D4 -> PCINT20 (PORTD group)
    {
        rightMotorPtr = m;

        PCICR  |= (1<<PCIE2);    // enable pin change for PORTD
        PCMSK2 |= (1<<PCINT20);  // enable PD4 pin change
        PCIFR  |= (1<<PCIF2);    // clear pending
    }
}

long motorGetCount(Motor *m)
{
    return m->count;
}

void motorResetCount(Motor *m)
{
    uint8_t s = SREG; cli();
    m->count = 0;
    SREG = s;
}

void motorForward(Motor *m, uint8_t duty_0_255)
{
    gpio_write(m->pinDir, 0);
    pwm_set_duty(m->pinPWM, duty_0_255);
}

void motorReverse(Motor *m, uint8_t duty_0_255)
{
    gpio_write(m->pinDir, 1);
    pwm_set_duty(m->pinPWM, duty_0_255);
}

void motorStop(Motor *m)
{
    pwm_set_duty(m->pinPWM, 0);

    // Set direction low
    gpio_write(m->pinDir, 0);

    // Put H-bridge into standby if shared
    gpio_write(m->pinStby, 1);
}
