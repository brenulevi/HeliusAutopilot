#ifndef HELIUS_TYPES_H
#define HELIUS_TYPES_H

typedef struct
{
    float gyro_x_rps;
    float gyro_y_rps;
    float gyro_z_rps;

    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
} helius_imu_t;

// Error state covariance matrix
typedef struct {
    float p[15][15];
} helius_cov15_t;

// Sensors noise parameters
typedef struct {
    float gyro_noise_std;   // rad/s/sqrt(Hz)
    float accel_noise_std;  // m/s^2/sqrt(Hz)
    float gyro_bias_std;    // rad/s^2/sqrt(Hz)
    float accel_bias_std;   // m/s^3/sqrt(Hz)
} helius_noise_config_t;

#endif