#include "kalmanFilter.h"

//stepup steps:
// 1. initalize once in setup()
//   kalmanPitch kf;
//   sample values to test and play around with:
//           float pinit   = 0.1f;  // set
//          float dt = 0.005f;
//          float q_angle = 0.001f; // tune this
//          float q_bias  = 0.01f;  // tune this
//          float r_angle = 0.5;    // tune this
// 2. in loop() read and convert in  all accelerometer xy and z values
//    and only the gyroscope x value (only acceleration around x axis is needed for pitch)
// 3. calculate pitch around x axis: atan2(ay, az) (in radians)
// 4. use kalmanPredict to do covariance prediciton
// 5. use kalmanUpdate to update measurement
// 6. kf.angle will be the filter result pitch/tilt angle!! :))))

static inline float wrap_pi(float a)
{
    // Wrap to (-pi, pi]
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a <= -M_PI) a += 2.0f * M_PI;
    return a;
}

// initalize filter
void initKalman(kalmanPitch *kal, float pinit, float q_angle, float q_bias, float r_ang)
{
    // State
    kal->angle = 0.0f;
    kal->bias  = 0.0f;

    // Covariance (diagonal start)
    kal->P[0] = pinit;  // P00
    kal->P[1] = 0.0f;   // P01
    kal->P[2] = 0.0f;   // P10
    kal->P[3] = pinit;  // P11

    // Noise parameters
    kal->q_angle = q_angle;
    kal->q_bias  = q_bias;
    kal->r_angle = r_ang;
}

// Euler/Riccati covariance prediction
// NOTE: gyro_deg_s is in DEGREES PER SECOND, angle is in DEGREES
void kalmanPredict(kalmanPitch *kal, float gyro_deg_s, float dt)
{
    // State predict: angle += (gyro - bias) * dt
    // angle [deg], gyro_deg_s [deg/s], bias [deg/s]
    kal->angle = kal->angle + (gyro_deg_s - kal->bias) * dt;

    // Cache P
    float P00 = kal->P[0], P01 = kal->P[1];
    float P10 = kal->P[2], P11 = kal->P[3];

    // Pdot = F P + P F' + Q, with F = [[0,-1],[0,0]], Q = diag(q_angle, q_bias)
    float P0 = kal->q_angle - P01 - P10; // d/dt P00
    float P1 = -P11;                     // d/dt P01
    float P2 = -P11;                     // d/dt P10
    float P3 =  kal->q_bias;             // d/dt P11

    // Euler integrate: P += Pdot * dt
    kal->P[0] = P00 + P0 * dt;  // P00
    kal->P[1] = P01 + P1 * dt;  // P01
    kal->P[2] = P10 + P2 * dt;  // P10
    kal->P[3] = P11 + P3 * dt;  // P11
}

// measurement update with tilt 
void kalmanUpdate(kalmanPitch *kal, float angle_meas_deg)
{
    // Innovation no wrap; work in small-angle deg range
    float error = angle_meas_deg - kal->angle;

    // PC' with C = [1 0]  →  [P00; P10]
    float PCt_0 = kal->P[0];  // P00
    float PCt_1 = kal->P[2];  // P10

    // S = R + C P C' = R + P00
    float E = kal->r_angle + PCt_0;

    // K = P C' S^{-1} → K = [P00/E; P10/E]
    float K0 = PCt_0 / E;
    float K1 = PCt_1 / E;

    float t0 = PCt_0;      // P00
    float t1 = kal->P[1];  // P01

    // P ← (I - K C) P
    kal->P[0] -= K0 * t0;  // P00
    kal->P[1] -= K0 * t1;  // P01
    kal->P[2] -= K1 * t0;  // P10
    kal->P[3] -= K1 * t1;  // P11

    // update state (no wrap)
    kal->angle = kal->angle + K0 * error;
    kal->bias  = kal->bias  + K1 * error;
}
