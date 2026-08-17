#include "ahrs.h"

#include <stddef.h>

#include "math/mat3.h"

static void extract_3x3_block_from_6x6(const mat6_t* src, mat3_t* dest, int row_offset, int col_offset)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            dest->data[i][j] = src->data[i + row_offset][j + col_offset];
        }
    }
}

static void insert_3x3_block_into_6x6(mat6_t* dest, const mat3_t* src, int row_offset, int col_offset)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            dest->data[i + row_offset][j + col_offset] = src->data[i][j];
        }
    }
}

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

    // Temporary matrices to use BSS section for performance
    static mat3_t P_tt, P_tb, P_bb;
    static mat3_t Phi, P_bb_new;
    static mat3_t W, P_tb_new, P_bt_new;
    static mat3_t P_tt_new, Phi_Pt, Phi_Pt_PhiT, W_T, Phi_T;

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
    extract_3x3_block_from_6x6(&ahrs->P, &P_tt, 0, 0);    // Orientation
    extract_3x3_block_from_6x6(&ahrs->P, &P_tb, 0, 3);    // Cross terms
    extract_3x3_block_from_6x6(&ahrs->P, &P_bb, 3, 3);    // Gyro bias

    // (Phi) F left upper block
    mat3_identity(&Phi);
    Phi.data[0][1] = gyro_corr.z * dt;
    Phi.data[0][2] = -gyro_corr.y * dt;
    Phi.data[1][0] = -gyro_corr.z * dt;
    Phi.data[1][2] = gyro_corr.x * dt;
    Phi.data[2][0] = gyro_corr.y * dt;
    Phi.data[2][1] = -gyro_corr.x * dt;

    // P_bb_new = P_bb + Q_bb
    mat3_copy(&P_bb, &P_bb_new);
    P_bb_new.data[0][0] += gyro_bias_variance;
    P_bb_new.data[1][1] += gyro_bias_variance;
    P_bb_new.data[2][2] += gyro_bias_variance;

    // P_tb_new = Phi * P_tb - dt * P_bb
    mat3_multiply(&Phi, &P_tb, &W);
    mat3_copy(&W, &P_tb_new);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            P_tb_new.data[i][j] -= dt * P_bb.data[i][j];
        }
    }

    // P_bt_new = P_tb_new^T
    mat3_transpose(&P_tb_new, &P_bt_new);

    // P_tt_new = Phi * P_tt * Phi^T - dt * W - dt * W^T + dt^2 * P_bb + Q_tt
    mat3_multiply(&Phi, &P_tt, &Phi_Pt);
    mat3_transpose(&Phi, &Phi_T);
    mat3_multiply(&Phi_Pt, &Phi_T, &Phi_Pt_PhiT);

    mat3_transpose(&W, &W_T);

    mat3_copy(&Phi_Pt_PhiT, &P_tt_new);
    float dt2 = dt * dt;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            P_tt_new.data[i][j] += -dt * W.data[i][j] - dt * W_T.data[i][j] + dt2 * P_bb.data[i][j];
        }
    }

    // Add process noise Q_tt
    P_tt_new.data[0][0] += gyro_noise_variance;
    P_tt_new.data[1][1] += gyro_noise_variance;
    P_tt_new.data[2][2] += gyro_noise_variance;

    // Update the 6x6 error covariance matrix P
    insert_3x3_block_into_6x6(&ahrs->P, &P_tt_new, 0, 0);
    insert_3x3_block_into_6x6(&ahrs->P, &P_tb_new, 0, 3);
    insert_3x3_block_into_6x6(&ahrs->P, &P_bt_new, 3, 0);
    insert_3x3_block_into_6x6(&ahrs->P, &P_bb_new, 3, 3);

    return AHRS_OK;
}

ahrs_status_t ahrs_update_accel(ahrs_t *ahrs, const vec3_t *accel_mps2, float dt)
{
    if (ahrs == NULL || accel_mps2 == NULL)
    {
        return AHRS_ERROR;
    }

    // Declare static matrices to avoid repeated allocations
    static mat3_t a_skew, a_skew_T;
    static mat6_t H;
    static mat3_t P_tt, P_bt;
    static mat3_t temp_sp, S, S_inv;
    static mat3_t Ptt_St, temp_Ktt;
    static mat3_t Pbt_St, temp_Kbb;
    static mat3_t Ktt_askew, Kbb_askeew;
    static mat6_t I_minus_KH;
    static mat6_t P_updated, temp1, I_minus_KH_T;
    static mat6_t K_R_KT;

    // Normalize the accelerometer measurement
    vec3_t accel_normalized = *accel_mps2;
    vec3_normalize(&accel_normalized);

    /* q maps body → inertial (right-multiply with body-frame δq).
     * Expected accelerometer direction is gravity in the body frame:
     * g_body = q* ⊗ g_inertial ⊗ q */
    vec3_t gravity_body;
    const vec3_t gravity_inertial = {0.0f, 0.0f, 1.0f};
    quat_rotate_vector_inverse(&ahrs->orientation, &gravity_inertial, &gravity_body);

    // Compute the innovation (y = z - h) for the accelerometer measurement
    vec3_t innovation;
    innovation.x = accel_normalized.x - gravity_body.x;
    innovation.y = accel_normalized.y - gravity_body.y;
    innovation.z = accel_normalized.z - gravity_body.z;

    // Compute the measurement Jacobian H for the accelerometer
    a_skew.data[0][0] = 0.0f;
    a_skew.data[0][1] = -gravity_body.z;
    a_skew.data[0][2] = gravity_body.y;

    a_skew.data[1][0] = gravity_body.z;
    a_skew.data[1][1] = 0.0f;
    a_skew.data[1][2] = -gravity_body.x;

    a_skew.data[2][0] = -gravity_body.y;
    a_skew.data[2][1] = gravity_body.x;
    a_skew.data[2][2] = 0.0f;

    /* g_body_true ≈ g_body + [g_body]× δθ  for q_true = q ⊗ δq, so H_θ = +[g]× */
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            H.data[i][j] = a_skew.data[i][j];
            H.data[i][j + 3] = 0.0f;
        }
    }

    // Calculate S = H * P * H^T + R
    // H * P * H^T affects only the top-left 3x3 block of P, so we can compute it directly
    extract_3x3_block_from_6x6(&ahrs->P, &P_tt, 0, 0);

    // S_θθ = [g]× P_tt [g]×^T  (sign of H cancels in H P H^T)
    mat3_multiply(&a_skew, &P_tt, &temp_sp);

    mat3_transpose(&a_skew, &a_skew_T);
    mat3_multiply(&temp_sp, &a_skew_T, &S);

    /* Snapshot measurement: R is not integrated over dt. After normalize,
     * this is treated as variance of the unit-vector components. */
    (void)dt;
    const float r_var = ahrs->config.accel_noise_density * ahrs->config.accel_noise_density;
    for (int i = 0; i < 3; ++i)
    {
        S.data[i][i] += r_var;
    }

    // Invert S (3x3 matrix)
    if (!mat3_inverse(&S, &S_inv))
    {
        return AHRS_ERROR;
    }

    // K = P H^T S^{-1} with H = [[g]×, 0], so H^T = [[g]×^T; 0]
    extract_3x3_block_from_6x6(&ahrs->P, &P_tt, 0, 0);
    extract_3x3_block_from_6x6(&ahrs->P, &P_bt, 3, 0);

    // K_tt = P_tt * a_skew^T * S_inv
    mat3_multiply(&P_tt, &a_skew_T, &Ptt_St);
    mat3_multiply(&Ptt_St, &S_inv, &temp_Ktt);

    // K_bb = P_bt * a_skew^T * S_inv
    mat3_multiply(&P_bt, &a_skew_T, &Pbt_St);
    mat3_multiply(&Pbt_St, &S_inv, &temp_Kbb);

    // Update the state estimate: x = x + K * y
    // Update orientation using the top part of K and the innovation
    // Update gyro bias using the bottom part of K and the innovation
    vec3_t delta_theta, delta_bias;

    delta_theta.x = temp_Ktt.data[0][0] * innovation.x + temp_Ktt.data[0][1] * innovation.y + temp_Ktt.data[0][2] * innovation.z;
    delta_theta.y = temp_Ktt.data[1][0] * innovation.x + temp_Ktt.data[1][1] * innovation.y + temp_Ktt.data[1][2] * innovation.z;
    delta_theta.z = temp_Ktt.data[2][0] * innovation.x + temp_Ktt.data[2][1] * innovation.y + temp_Ktt.data[2][2] * innovation.z;

    delta_bias.x = temp_Kbb.data[0][0] * innovation.x + temp_Kbb.data[0][1] * innovation.y + temp_Kbb.data[0][2] * innovation.z;
    delta_bias.y = temp_Kbb.data[1][0] * innovation.x + temp_Kbb.data[1][1] * innovation.y + temp_Kbb.data[1][2] * innovation.z;
    delta_bias.z = temp_Kbb.data[2][0] * innovation.x + temp_Kbb.data[2][1] * innovation.y + temp_Kbb.data[2][2] * innovation.z;

    // Update orientation
    // The innovation represents the correction accumulated over the time step, so we can apply it directly to the orientation quaternion.
    quat_t delta_q;
    delta_q.w = 1.0f;
    delta_q.x = 0.5f * delta_theta.x;
    delta_q.y = 0.5f * delta_theta.y;
    delta_q.z = 0.5f * delta_theta.z;
    
    quat_t new_orientation;
    quat_multiply(&ahrs->orientation, &delta_q, &new_orientation);
    quat_normalize(&new_orientation);
    ahrs->orientation = new_orientation;

    // Update gyro bias
    ahrs->gyro_bias.x += delta_bias.x;
    ahrs->gyro_bias.y += delta_bias.y;
    ahrs->gyro_bias.z += delta_bias.z;

    // Update the error covariance matrix using the Joseph form:
    // P = (I - K * H) * P * (I - K * H)^T + K * R * K^T

    // I - K H, with H = [[g]×, 0] so KH occupies the first three columns
    mat3_multiply(&temp_Ktt, &a_skew, &Ktt_askew);
    mat3_multiply(&temp_Kbb, &a_skew, &Kbb_askeew);

    mat6_identity(&I_minus_KH);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            I_minus_KH.data[i][j] -= Ktt_askew.data[i][j];
            I_minus_KH.data[i + 3][j] -= Kbb_askeew.data[i][j];
        }
    }

    // (I - KH) * P * (I - KH)^T
    // temp1 = (I - KH) * P
    mat6_multiply(&I_minus_KH, &ahrs->P, &temp1);
    
    mat6_transpose(&I_minus_KH, &I_minus_KH_T);
    mat6_multiply(&temp1, &I_minus_KH_T, &P_updated);
    
    // K * R * K^T
    mat6_zero(&K_R_KT);
    // R is r_var * I for the accelerometer measurement
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            K_R_KT.data[i][j] = temp_Ktt.data[i][0] * r_var * temp_Ktt.data[j][0] +
                                temp_Ktt.data[i][1] * r_var * temp_Ktt.data[j][1] +
                                temp_Ktt.data[i][2] * r_var * temp_Ktt.data[j][2];
            K_R_KT.data[i + 3][j + 3] = temp_Kbb.data[i][0] * r_var * temp_Kbb.data[j][0] +
                                        temp_Kbb.data[i][1] * r_var * temp_Kbb.data[j][1] +
                                        temp_Kbb.data[i][2] * r_var * temp_Kbb.data[j][2];
            K_R_KT.data[i][j + 3] = temp_Ktt.data[i][0] * r_var * temp_Kbb.data[j][0] +
                                    temp_Ktt.data[i][1] * r_var * temp_Kbb.data[j][1] +
                                    temp_Ktt.data[i][2] * r_var * temp_Kbb.data[j][2];
            K_R_KT.data[i + 3][j] = temp_Kbb.data[i][0] * r_var * temp_Ktt.data[j][0] +
                                    temp_Kbb.data[i][1] * r_var * temp_Ktt.data[j][1] +
                                    temp_Kbb.data[i][2] * r_var * temp_Ktt.data[j][2];
        }
    }

    // Update P
    mat6_add(&P_updated, &K_R_KT, &ahrs->P);

    // Ensure P remains symmetric
    for (int i = 0; i < 6; ++i)
    {
        for (int j = i + 1; j < 6; ++j)
        {
            float avg = 0.5f * (ahrs->P.data[i][j] + ahrs->P.data[j][i]);
            ahrs->P.data[i][j] = avg;
            ahrs->P.data[j][i] = avg;
        }
    }

    return AHRS_OK;
}
