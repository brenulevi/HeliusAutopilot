#include "estimator.h"
#include "mat.h"

#include <math.h>

#define ACCEL_MIN_NORM_MPS2 1e-3f
#define MAG_MIN_NORM_UT 1e-3f

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static bool project_onto_plane(vec3_t v, vec3_t plane_normal_hat, vec3_t* v_proj_hat);

static bool apply_vector_measurement_update(
    estimator_t* e,
    vec3_t meas_body_hat,
    vec3_t ref_nav_hat,
    float meas_dir_sigma,
    float nis_gate
)
{
    quat_t q_nb = quat_conjugate(e->attitude);
    vec3_t pred_body_hat = quat_rotate_vec3(q_nb, ref_nav_hat);
    if (!vec3_is_finite(pred_body_hat))
    {
        return false;
    }

    vec3_t residual = vec3_sub(meas_body_hat, pred_body_hat);

    float H[3][6] = {0};
    float H_theta[3][3] = {0};
    mat3_skew_from_vec3(pred_body_hat, H_theta);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            H[i][j] = H_theta[i][j];
        }
    }

    float Ht[6][3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            Ht[j][i] = H[i][j];
        }
    }

    float PHt[6][3] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                sum += e->P[i][k] * Ht[k][j];
            }
            PHt[i][j] = sum;
        }
    }

    float S[3][3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                sum += H[i][k] * PHt[k][j];
            }
            S[i][j] = sum;
        }
    }

    float sigma = fmaxf(meas_dir_sigma, e->min_meas_sigma);
    float R[3][3] = {
        {sigma * sigma, 0.0f, 0.0f},
        {0.0f, sigma * sigma, 0.0f},
        {0.0f, 0.0f, sigma * sigma},
    };
    for (int i = 0; i < 3; i++)
    {
        S[i][i] += R[i][i];
    }

    float S_inv[3][3] = {0};
    if (!mat3_inverse(S, S_inv))
    {
        return false;
    }

    if (nis_gate > 0.0f)
    {
        float Sr[3] = {0};
        float r[3] = {residual.x, residual.y, residual.z};
        for (int i = 0; i < 3; i++)
        {
            float sum = 0.0f;
            for (int j = 0; j < 3; j++)
            {
                sum += S_inv[i][j] * r[j];
            }
            Sr[i] = sum;
        }

        float nis = 0.0f;
        for (int i = 0; i < 3; i++)
        {
            nis += r[i] * Sr[i];
        }

        if (!isfinite(nis) || nis > nis_gate)
        {
            return false;
        }
    }

    float K[6][3] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                sum += PHt[i][k] * S_inv[k][j];
            }
            K[i][j] = sum;
        }
    }

    float dx[6] = {0};
    float residual_vec[3] = {residual.x, residual.y, residual.z};
    for (int i = 0; i < 6; i++)
    {
        float sum = 0.0f;
        for (int j = 0; j < 3; j++)
        {
            sum += K[i][j] * residual_vec[j];
        }
        dx[i] = sum;
    }

    vec3_t dtheta = {
        .x = dx[0],
        .y = dx[1],
        .z = dx[2],
    };
    quat_t dq = {
        .w = 1.0f,
        .x = 0.5f * dtheta.x,
        .y = 0.5f * dtheta.y,
        .z = 0.5f * dtheta.z,
    };
    dq = quat_normalize(dq);
    e->attitude = quat_mult(e->attitude, dq);
    e->attitude = quat_normalize(e->attitude);

    e->gyro_bias_rps.x = clampf(e->gyro_bias_rps.x + dx[3], -e->bias_limit_rps, e->bias_limit_rps);
    e->gyro_bias_rps.y = clampf(e->gyro_bias_rps.y + dx[4], -e->bias_limit_rps, e->bias_limit_rps);
    e->gyro_bias_rps.z = clampf(e->gyro_bias_rps.z + dx[5], -e->bias_limit_rps, e->bias_limit_rps);

    float KH[6][6] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                sum += K[i][k] * H[k][j];
            }
            KH[i][j] = sum;
        }
    }

    float I_KH[6][6] = {0};
    for (int i = 0; i < 6; i++)
    {
        I_KH[i][i] = 1.0f;
        for (int j = 0; j < 6; j++)
        {
            I_KH[i][j] -= KH[i][j];
        }
    }

    float temp[6][6] = {0};
    float I_KH_t[6][6] = {0};
    float joseph_left[6][6] = {0};
    mat6_mul(I_KH, e->P, temp);
    mat6_transpose(I_KH, I_KH_t);
    mat6_mul(temp, I_KH_t, joseph_left);

    float KR[6][3] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            KR[i][j] = K[i][j] * R[j][j];
        }
    }

    float KRKt[6][6] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                sum += KR[i][k] * K[j][k];
            }
            KRKt[i][j] = sum;
        }
    }

    mat6_add(joseph_left, KRKt, e->P);
    for (int i = 0; i < 6; i++)
    {
        e->P[i][i] = fmaxf(e->P[i][i], e->min_cov_diag);
    }
    mat6_symmetrize(e->P);
    return true;
}

static bool project_onto_plane(vec3_t v, vec3_t plane_normal_hat, vec3_t* v_proj_hat)
{
    vec3_t v_proj = vec3_sub(v, vec3_scale(plane_normal_hat, vec3_dot(v, plane_normal_hat)));
    float v_proj_norm = vec3_norm(v_proj);
    if (v_proj_norm <= 1e-6f)
    {
        return false;
    }

    *v_proj_hat = vec3_scale(v_proj, 1.0f / v_proj_norm);
    return true;
}

void estimator_init(estimator_t *e, float gyro_noise, float gyro_bias_noise)
{
    // That state vector initialization needs a better approach.
    // Maybe do some calibration first using accel/mag/gnss to get a good initial attitude.
    e->attitude = quat_identity();

    e->gyro_bias_rps.x = 0.0f;
    e->gyro_bias_rps.y = 0.0f;
    e->gyro_bias_rps.z = 0.0f;

    e->gyro_noise = gyro_noise;
    e->gyro_bias_noise = gyro_bias_noise;
    e->accel_noise = 0.20f;
    e->mag_noise = 0.1f;

    // Defaults are runtime-tunable via estimator_t fields by application code.
    e->accel_nis_gate = 9.21f;      // chi-square gate for 3 DoF at ~99%
    e->min_meas_sigma = 1e-4f;
    e->mag_jacobian_eps = 1e-4f;
    e->min_innovation_det = 1e-12f;
    e->min_cov_diag = 1e-6f;
    e->bias_limit_rps = 0.2f;

    e->mag_nav_ref = vec3_zero();
    e->mag_ref_valid = false;

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            e->P[i][j] = 0.0f;
        }
    }

    e->P[0][0] = 0.1f * 0.1f;
    e->P[1][1] = 0.1f * 0.1f;
    e->P[2][2] = 0.1f * 0.1f;
    e->P[3][3] = 0.01f * 0.01f;
    e->P[4][4] = 0.01f * 0.01f;
    e->P[5][5] = 0.01f * 0.01f;
}

void estimator_predict(estimator_t *e, const vec3_t* gyro_meas_rps, float dt)
{
    vec3_t gyro_corr = {
        .x = gyro_meas_rps->x - e->gyro_bias_rps.x,
        .y = gyro_meas_rps->y - e->gyro_bias_rps.y,
        .z = gyro_meas_rps->z - e->gyro_bias_rps.z,
    };

    quat_t dq = {
        .w = 1.0f,
        .x = 0.5f * gyro_corr.x * dt,
        .y = 0.5f * gyro_corr.y * dt,
        .z = 0.5f * gyro_corr.z * dt,
    };
    dq = quat_normalize(dq);

    // Integrate using Body -> Navigation
    e->attitude = quat_mult(dq, e->attitude);
    e->attitude = quat_normalize(e->attitude);

    // Jacobian calculation
    float F[6][6] = {0};
    for(int i = 0; i < 6; i++)
        F[i][i] = 1.0f;

    // Attitude relation
    F[0][1] = gyro_corr.z * dt;
    F[0][2] = -gyro_corr.y * dt;
    F[1][0] = -gyro_corr.z * dt;
    F[1][2] = gyro_corr.x * dt;
    F[2][0] = gyro_corr.y * dt;
    F[2][1] = -gyro_corr.x * dt;

    // Gyro bias relation
    F[0][3] = -dt;
    F[1][4] = -dt;
    F[2][5] = -dt;

    // Q matrix calculation
    float Q[6][6] = {0};
    
    float gyro_noise_sqr = e->gyro_noise * e->gyro_noise;
    float gyro_bias_noise_sqr = e->gyro_bias_noise * e->gyro_bias_noise;

    Q[0][0] = gyro_noise_sqr * dt;
    Q[1][1] = gyro_noise_sqr * dt;
    Q[2][2] = gyro_noise_sqr * dt;
    Q[3][3] = gyro_bias_noise_sqr * dt;
    Q[4][4] = gyro_bias_noise_sqr * dt;
    Q[5][5] = gyro_bias_noise_sqr * dt;

    float FP[6][6] = {0};
    float Ft[6][6] = {0};
    float FPFt[6][6] = {0};

    mat6_mul(F, e->P, FP);
    mat6_transpose(F, Ft);
    mat6_mul(FP, Ft, FPFt);
    mat6_add(FPFt, Q, e->P);
    mat6_symmetrize(e->P);
}

void estimator_update_accel(estimator_t* e, const vec3_t* accel_meas_mps2)
{
    if (!vec3_is_finite(*accel_meas_mps2))
    {
        return;
    }

    float accel_norm = vec3_norm(*accel_meas_mps2);
    if (accel_norm <= ACCEL_MIN_NORM_MPS2)
    {
        return;
    }

    vec3_t a_hat = vec3_scale(*accel_meas_mps2, 1.0f / accel_norm);

    vec3_t g_nav = {
        .x = 0.0f,
        .y = 0.0f,
        .z = -1.0f,
    };

    float accel_dir_sigma = fmaxf(e->accel_noise / accel_norm, e->min_meas_sigma);
    (void)apply_vector_measurement_update(e, a_hat, g_nav, accel_dir_sigma, e->accel_nis_gate);
}

void estimator_set_mag_reference(estimator_t* e, const vec3_t* mag_nav_ref)
{
    if (!vec3_is_finite(*mag_nav_ref))
    {
        e->mag_ref_valid = false;
        e->mag_nav_ref = vec3_zero();
        return;
    }

    float mag_ref_norm = vec3_norm(*mag_nav_ref);
    if (mag_ref_norm <= MAG_MIN_NORM_UT)
    {
        e->mag_ref_valid = false;
        e->mag_nav_ref = vec3_zero();
        return;
    }

    e->mag_nav_ref = vec3_scale(*mag_nav_ref, 1.0f / mag_ref_norm);
    e->mag_ref_valid = true;
}

void estimator_update_mag(estimator_t* e, const vec3_t* mag_meas)
{
    if (!e->mag_ref_valid || !vec3_is_finite(*mag_meas))
    {
        return;
    }

    float mag_norm = vec3_norm(*mag_meas);
    if (mag_norm <= MAG_MIN_NORM_UT)
    {
        return;
    }

    vec3_t mag_hat = vec3_scale(*mag_meas, 1.0f / mag_norm);
    vec3_t g_nav = {
        .x = 0.0f,
        .y = 0.0f,
        .z = -1.0f,
    };
    quat_t q_nb = quat_conjugate(e->attitude);
    vec3_t g_body_hat = quat_rotate_vec3(q_nb, g_nav);
    if (!vec3_is_finite(g_body_hat))
    {
        return;
    }

    vec3_t mag_body_horizontal_hat = vec3_zero();
    if (!project_onto_plane(mag_hat, g_body_hat, &mag_body_horizontal_hat))
    {
        return;
    }

    vec3_t mag_pred_body_hat = quat_rotate_vec3(q_nb, e->mag_nav_ref);
    if (!vec3_is_finite(mag_pred_body_hat))
    {
        return;
    }

    vec3_t mag_pred_horizontal_hat = vec3_zero();
    if (!project_onto_plane(mag_pred_body_hat, g_body_hat, &mag_pred_horizontal_hat))
    {
        return;
    }

    float meas2[2] = {
        mag_body_horizontal_hat.x,
        mag_body_horizontal_hat.y,
    };
    float pred2[2] = {
        mag_pred_horizontal_hat.x,
        mag_pred_horizontal_hat.y,
    };
    float residual2[2] = {
        meas2[0] - pred2[0],
        meas2[1] - pred2[1],
    };

    float H[2][6] = {0};
    const float eps = fmaxf(e->mag_jacobian_eps, 1e-8f);
    for (int j = 0; j < 3; j++)
    {
        vec3_t dtheta_eps = {
            .x = (j == 0) ? eps : 0.0f,
            .y = (j == 1) ? eps : 0.0f,
            .z = (j == 2) ? eps : 0.0f,
        };
        quat_t dq_eps = {
            .w = 1.0f,
            .x = 0.5f * dtheta_eps.x,
            .y = 0.5f * dtheta_eps.y,
            .z = 0.5f * dtheta_eps.z,
        };
        dq_eps = quat_normalize(dq_eps);

        quat_t q_pert = quat_mult(e->attitude, dq_eps);
        q_pert = quat_normalize(q_pert);
        quat_t q_pert_nb = quat_conjugate(q_pert);

        vec3_t g_body_hat_pert = quat_rotate_vec3(q_pert_nb, g_nav);
        if (!vec3_is_finite(g_body_hat_pert))
        {
            return;
        }

        vec3_t mag_pred_body_hat_pert = quat_rotate_vec3(q_pert_nb, e->mag_nav_ref);
        if (!vec3_is_finite(mag_pred_body_hat_pert))
        {
            return;
        }

        vec3_t mag_pred_horizontal_hat_pert = vec3_zero();
        if (!project_onto_plane(mag_pred_body_hat_pert, g_body_hat_pert, &mag_pred_horizontal_hat_pert))
        {
            return;
        }

        H[0][j] = (mag_pred_horizontal_hat_pert.x - pred2[0]) / eps;
        H[1][j] = (mag_pred_horizontal_hat_pert.y - pred2[1]) / eps;
    }

    float mag_dir_sigma = fmaxf(e->mag_noise / mag_norm, e->min_meas_sigma);
    float R2[2][2] = {
        {mag_dir_sigma * mag_dir_sigma, 0.0f},
        {0.0f, mag_dir_sigma * mag_dir_sigma},
    };

    float Ht[6][2] = {0};
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            Ht[j][i] = H[i][j];
        }
    }

    float PHt[6][2] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                sum += e->P[i][k] * Ht[k][j];
            }
            PHt[i][j] = sum;
        }
    }

    float S[2][2] = {0};
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                sum += H[i][k] * PHt[k][j];
            }
            S[i][j] = sum + R2[i][j];
        }
    }

    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (!(fabsf(det) > e->min_innovation_det) || !isfinite(det))
    {
        return;
    }

    float inv_det = 1.0f / det;
    float S_inv[2][2] = {
        { S[1][1] * inv_det, -S[0][1] * inv_det },
        { -S[1][0] * inv_det, S[0][0] * inv_det },
    };

    float K[6][2] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            float sum = 0.0f;
            for (int k = 0; k < 2; k++)
            {
                sum += PHt[i][k] * S_inv[k][j];
            }
            K[i][j] = sum;
        }
    }

    float dx[6] = {0};
    for (int i = 0; i < 6; i++)
    {
        dx[i] = K[i][0] * residual2[0] + K[i][1] * residual2[1];
    }

    vec3_t dtheta = {
        .x = dx[0],
        .y = dx[1],
        .z = dx[2],
    };
    quat_t dq = {
        .w = 1.0f,
        .x = 0.5f * dtheta.x,
        .y = 0.5f * dtheta.y,
        .z = 0.5f * dtheta.z,
    };
    dq = quat_normalize(dq);
    e->attitude = quat_mult(e->attitude, dq);
    e->attitude = quat_normalize(e->attitude);

    e->gyro_bias_rps.x = clampf(e->gyro_bias_rps.x + dx[3], -e->bias_limit_rps, e->bias_limit_rps);
    e->gyro_bias_rps.y = clampf(e->gyro_bias_rps.y + dx[4], -e->bias_limit_rps, e->bias_limit_rps);
    e->gyro_bias_rps.z = clampf(e->gyro_bias_rps.z + dx[5], -e->bias_limit_rps, e->bias_limit_rps);

    float I_KH[6][6] = {0};
    for (int i = 0; i < 6; i++)
    {
        I_KH[i][i] = 1.0f;
        for (int j = 0; j < 6; j++)
        {
            I_KH[i][j] -= K[i][0] * H[0][j] + K[i][1] * H[1][j];
        }
    }

    float temp[6][6] = {0};
    float I_KH_t[6][6] = {0};
    float joseph_left[6][6] = {0};
    mat6_mul(I_KH, e->P, temp);
    mat6_transpose(I_KH, I_KH_t);
    mat6_mul(temp, I_KH_t, joseph_left);

    float KR[6][2] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            KR[i][j] = K[i][0] * R2[0][j] + K[i][1] * R2[1][j];
        }
    }

    float KRKt[6][6] = {0};
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            KRKt[i][j] = KR[i][0] * K[j][0] + KR[i][1] * K[j][1];
        }
    }

    mat6_add(joseph_left, KRKt, e->P);
    for (int i = 0; i < 6; i++)
    {
        e->P[i][i] = fmaxf(e->P[i][i], e->min_cov_diag);
    }
    mat6_symmetrize(e->P);
}
