#include <avr/io.h>
#include <util/delay.h>
#include "imu.h"

// I2C clock 100kHz
#define F_CPU 16000000UL
#define SCL_CLOCK 100000L

static void i2c_init(void)
{
    TWSR = 0x00;
    TWBR = ((F_CPU / SCL_CLOCK) - 16) / 2;
}

static void i2c_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

static void i2c_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

static uint8_t i2c_read_ack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

static uint8_t i2c_read_nack(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

static void mpu_write(uint8_t reg, uint8_t val)
{
    i2c_start();
    i2c_write(MPU6050_ADDR << 1);  // write mode
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

static void mpu_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_start();
    i2c_write(MPU6050_ADDR << 1);
    i2c_write(reg);
    i2c_start();
    i2c_write((MPU6050_ADDR << 1) | 1); // read mode
    for (uint8_t i = 0; i < len - 1; i++)
        buf[i] = i2c_read_ack();
    buf[len - 1] = i2c_read_nack();
    i2c_stop();
}

void imu_init(void)
{
    i2c_init();
    _delay_ms(100);
    mpu_write(MPU6050_REG_PWR_MGMT_1, 0x01);  // wake up, clock = PLL with X gyro
    _delay_ms(100);

    // Match vendor defaults: 1 kHz sample, wide bandwidth
    mpu_write(MPU6050_REG_SMPLRT_DIV, 0x00);  // 1 kHz / (1 + 0) = 1000 Hz
    mpu_write(MPU6050_REG_CONFIG,      0x00); // DLPF = 260 Hz gyro, 256 Hz accel
    mpu_write(MPU6050_REG_GYRO_CONFIG, 0x00); // ±250°/s
    mpu_write(MPU6050_REG_ACCEL_CONFIG, 0x00);// ±2g
}

int16_t make16(uint8_t hi, uint8_t lo) { return (int16_t)((hi << 8) | lo); }

void imu_read(IMUData *data)
{
    uint8_t buf[14];
    mpu_read_bytes(MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    data->ax = make16(buf[0], buf[1]);
    data->ay = make16(buf[2], buf[3]);
    data->az = make16(buf[4], buf[5]);
    data->temp = make16(buf[6], buf[7]);
    data->gx = make16(buf[8], buf[9]);
    data->gy = make16(buf[10], buf[11]);
    data->gz = make16(buf[12], buf[13]);
}
