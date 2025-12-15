#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//changed to degrees
typedef struct {
    float angle;   // rad
    float bias;    // rad/s
    float P[4];    // covariance matrix
    float q_angle; // rad^2 / step
    float q_bias;  // (rad/s)^2 / step
    float r_angle; // rad^2
} kalmanPitch;

void initKalman(kalmanPitch *kal, float pinit, float q_angle, float q_bias, float r_ang);
void kalmanPredict(kalmanPitch *kal, float gyro_deg_s, float dt);
void kalmanUpdate(kalmanPitch *kal, float angle_meas_deg);

#ifdef __cplusplus
}
#endif

#endif // KALMANFILTER_H
