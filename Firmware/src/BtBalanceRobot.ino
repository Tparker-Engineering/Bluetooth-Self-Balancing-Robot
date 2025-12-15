#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>        
#include <stdbool.h>
#include <stdint.h>

#include "imu.h"
#include "kalmanFilter.h"
#include "motor.h"
#include "bluetooth.h"
#include "gpio.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define F_CPU 16000000UL   

#define AIN1               7
#define PWMA_LEFT          5
#define BIN1              12
#define PWMB_RIGHT         6
#define STBY_PIN           8
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_RIGHT_A_PIN 4

// ANGLE LIMITS
#define BALANCE_ANGLE_MIN_DEG (-37.0f)
#define BALANCE_ANGLE_MAX_DEG ( 37.0f)

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD (1.0f / RAD_TO_DEG)

// Control loop frequency
#define LOOP_FREQ_HZ 200.0f
#define DT (1.0f / LOOP_FREQ_HZ)

// Gyro scale raw -> rad/s  A±250 dps, 16-bit signed
#define GYRO_SCALE (250.0f / 32768.0f * M_PI / 180.0f)


static kalmanPitch kf;

// Angle state radians
volatile float accel_angle  = 0.0f;
volatile float kalman_angle = 0.0f;
volatile uint32_t loop_counter = 0;

volatile IMUData imu_data;

float angle_setpoint_deg = 0.0f;

int32_t gx_bias = 0;
float ANGLE_OFFSET_DEG = 0.0f;      // dynamically calibrated upright angle
volatile uint8_t fallen = 0;

// Balance angle PD gains 
float kp_balance = 55.0f;
float kd_balance = 0.75f;

// Speed PI gains 
float kp_speed   = 10.0f;
float ki_speed   = 0.26f;

// Turn gains yaw rotation control 
// Note: kp_turn is effectively "1" in code;
float kp_turn    = 0.7f;   // no real effect once rotation_control_out = turn_target + kd_turn * yaw_dps
float kd_turn    = 0.3f;

// Speed + turn setpoints controlled via Bluetooth 
// speed_target: >0 = forward, <0 = backward setting_car_speed
// turn_target:  >0 = turn left, <0 = turn right like setting_turn_speed
volatile float speed_target = 0.0f;   // encoder ticks per 40 ms
volatile float turn_target  = 0.0f;   // rotation magnitude

float speed_cmd = 0.0f;
float turn_cmd  = 0.0f;

// Speed loop state
volatile long   speed_sum_ticks   = 0;   // accumulated avg ticks over 5 ms steps
volatile uint8_t speed_count      = 0;   // counts 5 ms samples 
float           speed_filter      = 0.0f;
float           speed_filter_old  = 0.0f;
float           speed_integral    = 0.0f;
float           speed_control_out = 0.0f;

float last_speed_target = 0.0f;

// For debugging telemetry
float           balance_control_out = 0.0f;
float           total_control_out   = 0.0f;

// Telemetry control
volatile uint8_t telemetry_enabled = 0;   // 0 = off, 1 = on

static char cmd_buffer[32];
static uint8_t cmd_index = 0;

Motor leftMotor;
Motor rightMotor;

static uint8_t uart_available(void)
{
    // Non-zero if a byte has been received
    return (UCSR0A & (1 << RXC0));
}

// Forward declarations
static void handleCommand(const char *cmd);
static void printGains(void);

// Print all current gain values
static void printGains(void)
{
    char pb[16], db[16], ps[16], is[16], pt[16];
    char line[128];

    dtostrf(kp_balance, 6, 3, pb);
    dtostrf(kd_balance, 6, 3, db);
    dtostrf(kp_speed,   6, 3, ps);
    dtostrf(ki_speed,   6, 3, is);
    dtostrf(kp_turn,    6, 3, pt);

    snprintf(line, sizeof(line),
             "kp_balance=%s kd_balance=%s | kp_speed=%s ki_speed=%s | kp_turn=%s\r\n",
             pb, db, ps, is, pt);
    uart_puts(line);
}


// UART Accumulate a line and parse
static void process_uart_rx(void)
{
    while (uart_available())
    {
        char c = uart_getc();

        // Handle single motion commands immediately
        char uc = c;
        if (uc >= 'a' && uc <= 'z') uc = (char)(uc - 'a' + 'A');

        if (uc == 'F' || uc == 'B' || uc == 'S' ||
            uc == 'L' || uc == 'R' || uc == 'G' || uc == 'T')
        {
            char cmd[2] = { uc, '\0' };
            handleCommand(cmd);
            cmd_index = 0;   // clear any partial line
            continue;
        }

        // Line-based parsing for gain commands.
        if (c == '\r' || c == '\n')
        {
            if (cmd_index > 0)
            {
                cmd_buffer[cmd_index] = '\0';
                handleCommand(cmd_buffer);
                cmd_index = 0;
            }
        }
        else
        {
            if (cmd_index < (sizeof(cmd_buffer) - 1))
            {
                cmd_buffer[cmd_index++] = c;
            }
        }
    }
}


// Command line interface Bluetooth / UART
static void handleCommand(const char *cmd)
{
    while (*cmd == ' ' || *cmd == '\t')
        cmd++;

    if (!*cmd)
        return;

    // Normalize first two characters to uppercase
    char a = cmd[0];
    char b = cmd[1];

    if (a >= 'a' && a <= 'z') a = (char)(a - 'a' + 'A');
    if (b >= 'a' && b <= 'z') b = (char)(b - 'a' + 'A');

    // Gain set examples -> "PB 33", "DB 0.95", "PS 0.5", "IS 0.3", "PT 1.0"
    char which = 0;

    if (a == 'P' && b == 'B')      which = 'B';   // PB = kp_balance
    else if (a == 'D' && b == 'B') which = 'D';   // DB = kd_balance
    else if (a == 'P' && b == 'S') which = 'S';   // PS = kp_speed
    else if (a == 'I' && b == 'S') which = 'I';   // IS = ki_speed
    else if (a == 'P' && b == 'T') which = 'T';   // PT = kp_turn

    if (which)
    {
        const char *p = cmd + 2;
        while (*p == ' ' || *p == '\t') p++;

        float v = atof(p);
        char buf[64];

        if (which == 'B')   // PB = kp_balance
        {
            kp_balance = v;
            snprintf(buf, sizeof(buf), "kp_balance=%.3f\r\n", kp_balance);
            uart_puts(buf);
            printGains();
            gpio_toggle(13);
            return;
        }
        if (which == 'D')   // DB = kd_balance
        {
            kd_balance = v;
            snprintf(buf, sizeof(buf), "kd_balance=%.3f\r\n", kd_balance);
            uart_puts(buf);
            printGains();
            gpio_toggle(13);
            return;
        }
        if (which == 'S')   // PS = kp_speed
        {
            kp_speed = v;
            snprintf(buf, sizeof(buf), "kp_speed=%.3f\r\n", kp_speed);
            uart_puts(buf);
            printGains();
            gpio_toggle(13);
            return;
        }
        if (which == 'I')   // IS = ki_speed
        {
            ki_speed = v;
            snprintf(buf, sizeof(buf), "ki_speed=%.3f\r\n", ki_speed);
            uart_puts(buf);
            printGains();
            gpio_toggle(13);
            return;
        }
        if (which == 'T')   // PT = kp_turn
        {
            kp_turn = v;
            snprintf(buf, sizeof(buf), "kp_turn=%.3f\r\n", kp_turn);
            uart_puts(buf);
            printGains();
            gpio_toggle(13);
            return;
        }

        return;
    }

    // Treat the first non-space char as a movement command
    {
        char c = *cmd;

        // case-insensitive
        if (c >= 'a' && c <= 'z')
            c = (char)(c - 'a' + 'A');

        switch (c)
        {
            case 'F':   // forward 
                speed_target = +40.0f;   // like setting_car_speed = 80
                turn_target  = 0.0f;
                uart_puts("CMD: Forward\r\n");
                printGains();
                return;

            case 'B':   // backward
                speed_target = -40.0f;   // like setting_car_speed = -80 
                turn_target  = 0.0f;
                uart_puts("CMD: Backward\r\n");
                printGains();
                return;

            case 'S':   // stop movement keep balancing 
                speed_target      = 0.0f;
                turn_target       = 0.0f;

                uart_puts("CMD: Stop\r\n");
                return;

            case 'L':   // turn left in place 
                speed_target = 0.0f;
                turn_target  = +40.0f;   // like setting_turn_speed = +40 
                uart_puts("CMD: Turn-Left\r\n");
                printGains();
                return;

            case 'R':   // turn right in place
                speed_target = 0.0f;
                turn_target  = -40.0f;   // like setting_turn_speed = -40 
                uart_puts("CMD: Turn-Right\r\n");
                printGains();
                return;

            case 'G':   // show all gains 
                uart_puts("CMD: Show gains\r\n");
                printGains();
                return;

            case 'T':   // toggle telemetry on or off
            {
                telemetry_enabled = !telemetry_enabled;
                if (telemetry_enabled)
                    uart_puts("Telemetry: ON\r\n");
                else
                    uart_puts("Telemetry: OFF\r\n");
                return;
            }

            default:
                uart_puts("Unknown command\r\n");
                return;
        }
    }
}

// Timer1 compare interrupt at 200 Hz
ISR(TIMER1_COMPA_vect)
{
    loop_counter++;

    imu_read(&imu_data);

    // Accelerometer pitch angle from ay, az in DEGREES 
    float accel_angle_deg = atan2f((float)imu_data.ay, (float)imu_data.az) * RAD_TO_DEG;

    // Gyro X raw -> deg/s  (±250 dps, 16-bit signed)
    float gyro_dps = ((float)imu_data.gx - (float)gx_bias) * (250.0f / 32768.0f);

    // Kalman update state in DEGREES
    kalmanPredict(&kf, gyro_dps, DT);
    kalmanUpdate(&kf, accel_angle_deg);
    kalman_angle = kf.angle;   // kalman_angle is now in DEGREES

    float angle_deg = kalman_angle;   // pitch angle in degrees

    // Yaw rate Gyro Z in deg/s, Gyro_z = -gz / 131
    float yaw_dps = - (float)imu_data.gz * (250.0f / 32768.0f);  // deg/s

    // Encoder based speed ticks per 5 ms
    long dLeft  = leftMotor.count  - leftMotor.prevCount;
    long dRight = rightMotor.count - rightMotor.prevCount;
    leftMotor.prevCount  = leftMotor.count;
    rightMotor.prevCount = rightMotor.count;

    leftMotor.speed  = (float)dLeft;
    rightMotor.speed = (float)dRight;

    // Average forward speed signed
    float inst_speed = 0.5f * (leftMotor.speed + rightMotor.speed);

    // Accumulate for slower speed loop 40 ms
    speed_sum_ticks += (long)inst_speed;

    // Pitch relative to calibrated upright
    float pitch_deg = angle_deg - ANGLE_OFFSET_DEG;

    // ANGLE LIMITS 
    if (pitch_deg < BALANCE_ANGLE_MIN_DEG || pitch_deg > BALANCE_ANGLE_MAX_DEG)
    {
        // Consider it fallen out of balance range
        fallen = 1;

        speed_target     = 0.0f;
        turn_target      = 0.0f;
        speed_integral   = 0.0f;
        speed_filter     = 0.0f;
        speed_filter_old = 0.0f;
        speed_cmd        = 0.0f;
        turn_cmd         = 0.0f;
        speed_control_out = 0.0f;

        motorStop(&leftMotor);
        motorStop(&rightMotor);

        leftMotor.count      = 0;
        rightMotor.count     = 0;
        leftMotor.prevCount  = 0;
        rightMotor.prevCount = 0;

        return;
    }
    else
    {
        // Re-enable when upright again < 10 degrees from upright
        if (fallen && fabsf(pitch_deg) < 10.0f)
        {
            fallen = 0;
            speed_integral   = 0.0f;
            speed_filter     = 0.0f;
            speed_filter_old = 0.0f;

            leftMotor.count      = 0;
            rightMotor.count     = 0;
            leftMotor.prevCount  = 0;
            rightMotor.prevCount = 0;

            // Re-enable driver standby pins 
            gpio_write(leftMotor.pinStby,  1);
            gpio_write(rightMotor.pinStby, 1);
        }
    }

    const float SPEED_ALPHA = 0.1f;  // 0..1, smaller = slower response

    speed_cmd = (1.0f - SPEED_ALPHA) * speed_cmd + SPEED_ALPHA * speed_target;
    turn_cmd  = (1.0f - SPEED_ALPHA) * turn_cmd  + SPEED_ALPHA * turn_target;

    if ((speed_target > 0.0f && last_speed_target <= 0.0f) ||
        (speed_target < 0.0f && last_speed_target >= 0.0f) ||
        fabsf(speed_target - last_speed_target) > 20.0f)    // large speed change
    {
        speed_integral    = 0.0f;
        speed_filter      = 0.0f;
        speed_filter_old  = 0.0f;
        speed_sum_ticks   = 0;
        speed_count       = 0;
    }
    last_speed_target = speed_target;

    // Outer speed PI loop every 8 * 5 ms = 40 ms 
    if (++speed_count >= 8)
    {
        speed_count = 0;

        // total ticks over last 40 ms
        float car_speed = (float)speed_sum_ticks;
        speed_sum_ticks = 0;

        // low-pass filter of measured speed
        speed_filter = 0.7f * speed_filter_old + 0.3f * car_speed;
        speed_filter_old = speed_filter;

        // Standard speed error
        float speed_error = speed_target - speed_filter;

        // Integrate error
        speed_integral += speed_error;

        // clamp integral
        if (speed_integral > 3000.0f)  speed_integral = 3000.0f;
        if (speed_integral < -3000.0f) speed_integral = -3000.0f;

        // PI output
        speed_control_out = kp_speed * speed_error + ki_speed * speed_integral;
    }

    // Balance PD angle + gyro
    float angle_error = angle_deg - ANGLE_OFFSET_DEG;
    float gyro_error  = gyro_dps;

    // Negative sign so a positive angle_error produces the correct wheel motion
    balance_control_out =
        -kp_balance * angle_error   // P term
        -kd_balance * gyro_error;   // D term

    // Base control = balance + speed PI output
    total_control_out = balance_control_out + speed_control_out;

    // ROTATION CONTROL:
    float rotation_control_out = turn_cmd + kd_turn * yaw_dps;

    // Mixing:
    float left_cmd  = total_control_out - rotation_control_out;
    float right_cmd = total_control_out + rotation_control_out;

    // Drive left motor
    int pwm_left = (int)fabsf(left_cmd);
    if (pwm_left > 0 && pwm_left < 15) pwm_left = 15;  // deadband
    if (pwm_left > 255)               pwm_left = 255;

    if (left_cmd > 0.0f)
        motorReverse(&leftMotor, pwm_left);
    else
        motorForward(&leftMotor, pwm_left);

    // Drive right motor
    int pwm_right = (int)fabsf(right_cmd);
    if (pwm_right > 0 && pwm_right < 15) pwm_right = 15;
    if (pwm_right > 255)                pwm_right = 255;

    if (right_cmd > 0.0f)
        motorReverse(&rightMotor, pwm_right);
    else
        motorForward(&rightMotor, pwm_right);
}

// Timer setup for 200 Hz interrupt
void timer1_init(void)
{
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);                  // CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10);     // /64 prescaler
    OCR1A = 1249;                            // 5 ms at 16 MHz
    TIMSK1 |= (1 << OCIE1A);                 // enable compare match interrupt
}


// Main
int main(void)
{
    gpio_pinMode(13, GPIO_OUTPUT);
    gpio_pinMode(12, GPIO_OUTPUT);

    uart_init();        
    imu_init();

    // initKalman(kf, pinit, q_angle, q_bias, r_angle)
    initKalman(&kf, 0.1f, 0.001f, 0.005f, 0.5f);

    motorInit(&leftMotor,  AIN1, PWMA_LEFT, STBY_PIN, ENCODER_LEFT_A_PIN);
    motorInit(&rightMotor, BIN1, PWMB_RIGHT, STBY_PIN, ENCODER_RIGHT_A_PIN);

    _delay_ms(1000);

    // Gyro bias calibration X
    int32_t sum = 0;
    for (int i = 0; i < 500; i++)
    {
        imu_read(&imu_data);
        sum += imu_data.gx;
        _delay_ms(2);
    }
    gx_bias = sum / 500;

    // Angle offset calibration
    {
        float sum_ang = 0.0f;
        for (int i = 0; i < 500; i++)
        {
            imu_read(&imu_data);
            float a = atan2f((float)imu_data.ay, (float)imu_data.az);
            sum_ang += a;
            _delay_ms(2);
        }
        float avg_ang = sum_ang / 500.0f;
        ANGLE_OFFSET_DEG = avg_ang * RAD_TO_DEG;
    }

    uart_puts("Kalman balance robot starting...\r\n");
    printGains();

    timer1_init();
    sei();

    uint32_t last_telemetry = 0;

    while (1)
    {
        process_uart_rx();

        if (telemetry_enabled)
        {
            uint32_t now = loop_counter;
            if ((now - last_telemetry) >= 20) // 100 ms
            {
                last_telemetry = now;

                float pitch_deg = kalman_angle - ANGLE_OFFSET_DEG;

                char msg[128];
                char pitch_str[16];
                char ctrl_str[16];
                char speed_str[16];

                dtostrf(pitch_deg,         6, 2, pitch_str);   // pitch in deg
                dtostrf(total_control_out, 6, 1, ctrl_str);    // total control
                dtostrf(speed_filter,      6, 1, speed_str);   // filtered speed

                snprintf(msg, sizeof(msg),
                         "Pitch=%s deg | Ctrl=%s | Spd=%s\r\n",
                         pitch_str, ctrl_str, speed_str);
                uart_puts(msg);
            }
        }
    }
}