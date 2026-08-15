#ifndef HELIUS_AHRS_H
#define HELIUS_AHRS_H

#include <stdbool.h>

#include "math/quat.h"
#include "math/vec3.h"
#include "math/mat6.h"

typedef enum
{
    AHRS_OK = 0,
    AHRS_ERROR = 1,
} ahrs_status_t;

typedef struct
{
    float initial_orientation_std; // Standard deviation of the initial orientation (in radians)
    float initial_gyro_bias_std;   // Standard deviation of the initial gyroscope bias (in radians/sec)

    float gyro_noise_density; // Gyroscope noise density (in radians/sec/sqrt(Hz))
    float gyro_bias_random_walk; // Gyroscope bias random walk (in radians/sec^2/sqrt(Hz))
    float accel_noise_density; // Accelerometer noise density (in m/s^2/sqrt(Hz))
} ahrs_config_t;

typedef struct
{
    // State variables
    quat_t orientation;
    vec3_t gyro_bias;

    mat6_t P; // Error covariance matrix

    ahrs_config_t config;

    bool is_initialized;
} ahrs_t;

ahrs_status_t ahrs_init(ahrs_t *ahrs, const ahrs_config_t *config);
ahrs_status_t ahrs_predict(ahrs_t *ahrs, const vec3_t *gyro_rps, float dt);
ahrs_status_t ahrs_update_accel(ahrs_t *ahrs, const vec3_t *accel_mps2, float dt);

#endif // HELIUS_AHRS_H