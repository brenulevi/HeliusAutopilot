#ifndef estimator_H
#define estimator_H

#include "math/quat.h"
#include "math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    quat_t attitude;
    vec3_t gyro_bias_rps;
    vec3_t mag_nav_ref;

    float P[6][6];

    float gyro_noise;
    float gyro_bias_noise;
    float accel_noise;
    float mag_noise;

    // EKF tuning/limits (runtime-configurable defaults set in estimator_init)
    float accel_nis_gate;
    float min_meas_sigma;
    float mag_jacobian_eps;
    float min_innovation_det;
    float min_cov_diag;
    float bias_limit_rps;

    bool mag_ref_valid;
} estimator_t;

void estimator_init(
    estimator_t* e,
    float gyro_noise,
    float gyro_bias_noise
);
void estimator_predict(
    estimator_t* e,
    const vec3_t* gyro_meas_rps,
    float dt
);
void estimator_update_accel(
    estimator_t* e,
    const vec3_t* accel_meas_mps2
);
void estimator_set_mag_reference(
    estimator_t* e,
    const vec3_t* mag_nav_ref
);
void estimator_update_mag(
    estimator_t* e,
    const vec3_t* mag_meas
);

#ifdef __cplusplus
}
#endif

#endif