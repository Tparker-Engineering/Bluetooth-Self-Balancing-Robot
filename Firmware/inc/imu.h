#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// MPU-6050 I2C address (AD0 = GND 0x68)
#define MPU6050_ADDR  0x68
// MPU-6050 register map
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_TEMP_OUT_H   0x41

typedef struct
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
} IMUData;

#ifdef __cplusplus
extern "C" {
#endif

void imu_init(void);
void imu_read(IMUData *data);

#ifdef __cplusplus
}
#endif

#endif
