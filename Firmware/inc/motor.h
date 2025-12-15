#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef struct
{
    uint8_t pinDir;      // direction pin (digital)
    uint8_t pinPWM;      // PWM pin (must be a hardware PWM pin: 3,5,6,9,10,11)
    uint8_t pinStby;     // standby/enable pin (digital)
    uint8_t pinEnc;      // encoder input pin (left: 2/INT0, right: 4/PCINT20)
    volatile long count; // pulse count (ISR updated)
    long prevCount;      // last count snapshot
    float speed;         // computed wheel speed
} Motor;

#ifdef __cplusplus
extern "C" {
#endif

void motorInit(Motor *m, uint8_t dir, uint8_t pwm, uint8_t stby, uint8_t pinEnc);
void motorForward(Motor *m, uint8_t duty_0_255);
void motorReverse(Motor *m, uint8_t duty_0_255);
void motorStop(Motor *m);
long motorGetCount(Motor *m);
void motorResetCount(Motor *m);

#ifdef __cplusplus
}
#endif

#endif
