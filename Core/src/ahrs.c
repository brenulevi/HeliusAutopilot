#include "ahrs.h"

#include <stddef.h>

ahrs_status_t ahrs_init(ahrs_t* ahrs, const ahrs_config_t* config)
{
    if (ahrs == NULL || config == NULL)
    {
        return AHRS_ERROR;
    }
    
    // Initialize state variables
    quat_identity(&ahrs->orientation);
    vec3_zero(&ahrs->gyro_bias);

    ahrs->config = *config;

    // Initialize error covariance matrix P
    mat6_zero(&ahrs->P);

    const float initial_orientation_variance = config->initial_orientation_std * config->initial_orientation_std;
    const float initial_gyro_bias_variance = config->initial_gyro_bias_std * config->initial_gyro_bias_std;

    ahrs->P.data[0][0] = initial_orientation_variance;
    ahrs->P.data[1][1] = initial_orientation_variance;
    ahrs->P.data[2][2] = initial_orientation_variance;

    ahrs->P.data[3][3] = initial_gyro_bias_variance;
    ahrs->P.data[4][4] = initial_gyro_bias_variance;
    ahrs->P.data[5][5] = initial_gyro_bias_variance;

    ahrs->is_initialized = true;
    
    return AHRS_OK;
}

ahrs_status_t ahrs_predict(ahrs_t *ahrs, const vec3_t *gyro_rps, float dt)
{
    if (ahrs == NULL || gyro_rps == NULL || dt <= 0.0f)
    {
        return AHRS_ERROR;
    }

    vec3_t gyro_corr;
    gyro_corr.x = gyro_rps->x - ahrs->gyro_bias.x;
    gyro_corr.y = gyro_rps->y - ahrs->gyro_bias.y;
    gyro_corr.z = gyro_rps->z - ahrs->gyro_bias.z;

    // Predict the new orientation using the corrected gyroscope measurements
    quat_t delta_q;
    float half_dt = 0.5f * dt;
    delta_q.w = 1.0f;
    delta_q.x = half_dt * gyro_corr.x;
    delta_q.y = half_dt * gyro_corr.y;
    delta_q.z = half_dt * gyro_corr.z;

    quat_t new_orientation;
    quat_multiply(&ahrs->orientation, &delta_q, &new_orientation);
    quat_normalize(&new_orientation);
    ahrs->orientation = new_orientation;

    // Calculate the jacobian F for the state transition
    mat6_t F;
    mat6_identity(&F);
    
    /* F attitude block ≈ I + [-ω×] dt  (error-state) */
    F.data[0][1] = gyro_corr.z * dt;
    F.data[0][2] = -gyro_corr.y * dt;
    F.data[1][0] = -gyro_corr.z * dt;
    F.data[1][2] = gyro_corr.x * dt;
    F.data[2][0] = gyro_corr.y * dt;
    F.data[2][1] = -gyro_corr.x * dt;

    F.data[0][3] = -dt;
    F.data[1][4] = -dt;
    F.data[2][5] = -dt;

    // Calculate the process noise covariance Q
    mat6_t Q;
    mat6_zero(&Q);

    const float gyro_noise_variance = ahrs->config.gyro_noise_density * ahrs->config.gyro_noise_density * dt;
    const float gyro_bias_variance = ahrs->config.gyro_bias_random_walk * ahrs->config.gyro_bias_random_walk * dt;

    Q.data[0][0] = gyro_noise_variance;
    Q.data[1][1] = gyro_noise_variance;
    Q.data[2][2] = gyro_noise_variance;

    Q.data[3][3] = gyro_bias_variance;
    Q.data[4][4] = gyro_bias_variance;
    Q.data[5][5] = gyro_bias_variance;

    // Update the error covariance matrix P = F * P * F^T + Q
    mat6_t FP;
    mat6_t Ft;
    mat6_t FPFt;
    mat6_t new_P;

    mat6_transpose(&F, &Ft);
    mat6_multiply(&F, &ahrs->P, &FP);
    mat6_multiply(&FP, &Ft, &FPFt);
    mat6_add(&FPFt, &Q, &new_P);
    ahrs->P = new_P;

    return AHRS_OK;
}
