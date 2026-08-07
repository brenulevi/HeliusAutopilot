#include "estimator.h"
#include <stddef.h>
#include <string.h>

#define DEFAULT_GYRO_NOISE      2.44e-4f // rad/s/sqrt(Hz)
#define DEFAULT_ACCEL_NOISE     1.57e-3f // m/s^2/sqrt(Hz)
#define DEFAULT_GYRO_BIAS       1.00e-5f // rad/s^2/sqrt(Hz)
#define DEFAULT_ACCEL_BIAS      1.00e-4f // m/s^3/sqrt(Hz)

void helius_estimator_init(helius_estimator_t *estimator, const helius_noise_config_t* noise_cfg)
{
    if (estimator == NULL) return;

    estimator->attitude_rad.w = 1.0f;
    estimator->attitude_rad.x = 0.0f;
    estimator->attitude_rad.y = 0.0f;
    estimator->attitude_rad.z = 0.0f;

    estimator->position_ned = (helius_vec3_t){0.0f, 0.0f, 0.0f};
    estimator->velocity_mps = (helius_vec3_t){0.0f, 0.0f, 0.0f};

    estimator->gyro_bias_rps  = (helius_vec3_t){0.0f, 0.0f, 0.0f};
    estimator->accel_bias_mps2 = (helius_vec3_t){0.0f, 0.0f, 0.0f};

    estimator->a_nav_prev = (helius_vec3_t){0.0f, 0.0f, 0.0f};

    if (noise_cfg != NULL) {
        estimator->noise = *noise_cfg;
    } else {
        estimator->noise.gyro_noise_std  = DEFAULT_GYRO_NOISE; 
        estimator->noise.accel_noise_std = DEFAULT_ACCEL_NOISE; 
        estimator->noise.gyro_bias_std   = DEFAULT_GYRO_BIAS; 
        estimator->noise.accel_bias_std  = DEFAULT_ACCEL_BIAS; 
    }

    // Direct pointers to memory buffers
    estimator->P = &estimator->P_buffer_a;
    estimator->P_pred = &estimator->P_buffer_b;

    // Clear both buffers
    memset(&estimator->P_buffer_a, 0, sizeof(helius_cov15_t));
    memset(&estimator->P_buffer_b, 0, sizeof(helius_cov15_t));

    // Initial variances setup using estimator->P
    float init_var_att_xy = 0.087f * 0.087f; 
    float init_var_att_z  = 0.523f * 0.523f; 
    float init_var_pos    = 10.0f  * 10.0f; 
    float init_var_vel    = 1.0f   * 1.0f; 
    float init_var_bg     = 0.05f  * 0.05f; 
    float init_var_ba     = 0.2f   * 0.2f; 

    estimator->P->p[0][0] = init_var_att_xy;
    estimator->P->p[1][1] = init_var_att_xy;
    estimator->P->p[2][2] = init_var_att_z;

    estimator->P->p[3][3] = init_var_pos;
    estimator->P->p[4][4] = init_var_pos;
    estimator->P->p[5][5] = init_var_pos;

    estimator->P->p[6][6] = init_var_vel;
    estimator->P->p[7][7] = init_var_vel;
    estimator->P->p[8][8] = init_var_vel;

    estimator->P->p[9][9]   = init_var_bg;
    estimator->P->p[10][10] = init_var_bg;
    estimator->P->p[11][11] = init_var_bg;

    estimator->P->p[12][12] = init_var_ba;
    estimator->P->p[13][13] = init_var_ba;
    estimator->P->p[14][14] = init_var_ba;

    estimator->is_initialized = 1;
}

static void predict_nominal_state(
    helius_estimator_t* estimator, 
    helius_vec3_t w_corr, 
    helius_vec3_t a_corr, 
    float dt) 
{
    // Integrate gyro rates to get updated attitude using Exponential Quaternion Method
    helius_quat_t dq = helius_quat_from_axis_angle(w_corr, dt);
    estimator->attitude_rad = helius_quat_multiply(estimator->attitude_rad, dq);
    estimator->attitude_rad = helius_quat_normalize(estimator->attitude_rad);

    // Rotate accel to NED
    helius_vec3_t a_nav_curr = helius_quat_rotate_vec(estimator->attitude_rad, a_corr);

    // Remove gravity from accel to just appear specific acceleration
    a_nav_curr.z -= HELIUS_GRAVITY;

    // Calculate average acceleration
    helius_vec3_t a_nav_avg = {
        0.5f * (a_nav_curr.x + estimator->a_nav_prev.x),
        0.5f * (a_nav_curr.y + estimator->a_nav_prev.y),
        0.5f * (a_nav_curr.z + estimator->a_nav_prev.z)};

    // Integrate velocity to get position
    estimator->position_ned.x += estimator->velocity_mps.x * dt + 0.5f * a_nav_avg.x * dt * dt;
    estimator->position_ned.y += estimator->velocity_mps.y * dt + 0.5f * a_nav_avg.y * dt * dt;
    estimator->position_ned.z += estimator->velocity_mps.z * dt + 0.5f * a_nav_avg.z * dt * dt;

    // Integrate accel to get velocity
    estimator->velocity_mps.x += a_nav_avg.x * dt;
    estimator->velocity_mps.y += a_nav_avg.y * dt;
    estimator->velocity_mps.z += a_nav_avg.z * dt;

    // Update previous NED acceleration
    estimator->a_nav_prev = a_nav_curr;
}

// =============================================================================
// HELPER FUNCTIONS FOR 3x3 BLOCK MATRIX OPERATIONS
// =============================================================================

static inline helius_mat3_t block_get(const helius_cov15_t* P, uint8_t row_idx, uint8_t col_idx) {
    helius_mat3_t block;
    uint8_t r_off = row_idx * 3;
    uint8_t c_off = col_idx * 3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            block.m[i][j] = P->p[r_off + i][c_off + j];
        }
    }
    return block;
}

static inline void block_set(helius_cov15_t* P, uint8_t row_idx, uint8_t col_idx, const helius_mat3_t* block) {
    uint8_t r_off = row_idx * 3;
    uint8_t c_off = col_idx * 3;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P->p[r_off + i][c_off + j] = block->m[i][j];
        }
    }
}

static inline helius_mat3_t mat3_add(helius_mat3_t a, helius_mat3_t b) {
    helius_mat3_t res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.m[i][j] = a.m[i][j] + b.m[i][j];
        }
    }
    return res;
}

static inline helius_mat3_t mat3_scale(helius_mat3_t a, float scalar) {
    helius_mat3_t res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.m[i][j] = a.m[i][j] * scalar;
        }
    }
    return res;
}

// =============================================================================
// ERROR-STATE COVARIANCE PROPAGATION (15x15 via 3x3 Blocks)
// =============================================================================

static void predict_covariance(
    helius_estimator_t* estimator, 
    helius_vec3_t w_corr, 
    helius_vec3_t a_corr, 
    float dt) 
{
    // 1. Compute rotation matrix
    helius_mat3_t R_body_to_nav = helius_mat3_from_quat(estimator->attitude_rad);

    // 2. Build non-zero 3x3 sub-blocks of state transition matrix Phi
    helius_mat3_t I3 = helius_mat3_identity();
    helius_mat3_t w_skew = helius_mat3_skew(w_corr);
    helius_mat3_t Phi_00 = mat3_add(I3, mat3_scale(w_skew, -dt));
    helius_mat3_t Phi_03 = mat3_scale(I3, -dt);
    helius_mat3_t Phi_12 = mat3_scale(I3, dt);

    helius_mat3_t a_skew = helius_mat3_skew(a_corr);
    helius_mat3_t R_a_skew = helius_mat3_mult(R_body_to_nav, a_skew);
    helius_mat3_t Phi_20 = mat3_scale(R_a_skew, -dt);
    helius_mat3_t Phi_24 = mat3_scale(R_body_to_nav, -dt);

    helius_mat3_t Phi_00_T = helius_mat3_transpose(Phi_00);
    helius_mat3_t Phi_03_T = helius_mat3_transpose(Phi_03);
    helius_mat3_t Phi_12_T = helius_mat3_transpose(Phi_12);
    helius_mat3_t Phi_20_T = helius_mat3_transpose(Phi_20);
    helius_mat3_t Phi_24_T = helius_mat3_transpose(Phi_24);

    // Buffer temporário leve apenas para uma linha (180 bytes na pilha)
    helius_mat3_t M_row[HELIUS_BLOCK_COUNT];

    for (int i = 0; i < HELIUS_BLOCK_COUNT; i++) {
        // Step A: Calculate Row i of M = Phi * P_active
        for (int j = 0; j < HELIUS_BLOCK_COUNT; j++) {
            helius_mat3_t P_0j = block_get(estimator->P, HELIUS_BLOCK_ATT, j);
            helius_mat3_t P_1j = block_get(estimator->P, HELIUS_BLOCK_POS, j);
            helius_mat3_t P_2j = block_get(estimator->P, HELIUS_BLOCK_VEL, j);
            helius_mat3_t P_3j = block_get(estimator->P, HELIUS_BLOCK_BG,  j);
            helius_mat3_t P_4j = block_get(estimator->P, HELIUS_BLOCK_BA,  j);

            switch (i) {
                case HELIUS_BLOCK_ATT:
                    M_row[j] = mat3_add(helius_mat3_mult(Phi_00, P_0j), helius_mat3_mult(Phi_03, P_3j));
                    break;
                case HELIUS_BLOCK_POS:
                    M_row[j] = mat3_add(P_1j, helius_mat3_mult(Phi_12, P_2j));
                    break;
                case HELIUS_BLOCK_VEL:
                    M_row[j] = mat3_add(helius_mat3_mult(Phi_20, P_0j), mat3_add(P_2j, helius_mat3_mult(Phi_24, P_4j)));
                    break;
                case HELIUS_BLOCK_BG:
                    M_row[j] = P_3j;
                    break;
                case HELIUS_BLOCK_BA:
                    M_row[j] = P_4j;
                    break;
            }
        }

        // Step B: Calculate Row i of P_pred = M_row * Phi^T
        for (int j = i; j < HELIUS_BLOCK_COUNT; j++) {
            helius_mat3_t block_ij;

            switch (j) {
                case HELIUS_BLOCK_ATT:
                    block_ij = mat3_add(
                        helius_mat3_mult(M_row[HELIUS_BLOCK_ATT], Phi_00_T), 
                        helius_mat3_mult(M_row[HELIUS_BLOCK_BG],  Phi_03_T)
                    );
                    break;
                case HELIUS_BLOCK_POS:
                    block_ij = mat3_add(
                        M_row[HELIUS_BLOCK_POS], 
                        helius_mat3_mult(M_row[HELIUS_BLOCK_VEL], Phi_12_T)
                    );
                    break;
                case HELIUS_BLOCK_VEL:
                    block_ij = mat3_add(
                        helius_mat3_mult(M_row[HELIUS_BLOCK_ATT], Phi_20_T),
                        mat3_add(M_row[HELIUS_BLOCK_VEL], helius_mat3_mult(M_row[HELIUS_BLOCK_BA], Phi_24_T))
                    );
                    break;
                case HELIUS_BLOCK_BG:
                    block_ij = M_row[HELIUS_BLOCK_BG];
                    break;
                case HELIUS_BLOCK_BA:
                    block_ij = M_row[HELIUS_BLOCK_BA];
                    break;
            }

            // Write to predicted buffer
            block_set(estimator->P_pred, i, j, &block_ij);

            if (i != j) {
                helius_mat3_t block_ji = helius_mat3_transpose(block_ij);
                block_set(estimator->P_pred, j, i, &block_ji);
            }
        }
    }

    // 4. Add Process Noise (Qd) directly to P_pred diagonal
    float gyro_var_dt   = estimator->noise.gyro_noise_std  * estimator->noise.gyro_noise_std  * dt;
    float accel_var_dt  = estimator->noise.accel_noise_std * estimator->noise.accel_noise_std * dt;
    float g_bias_var_dt = estimator->noise.gyro_bias_std   * estimator->noise.gyro_bias_std   * dt;
    float a_bias_var_dt = estimator->noise.accel_bias_std  * estimator->noise.accel_bias_std  * dt;

    for (int k = 0; k < 3; k++) {
        estimator->P_pred->p[0 + k][0 + k]   += gyro_var_dt;
        estimator->P_pred->p[6 + k][6 + k]   += accel_var_dt;
        estimator->P_pred->p[9 + k][9 + k]   += g_bias_var_dt;
        estimator->P_pred->p[12 + k][12 + k] += a_bias_var_dt;
    }

    // 5. Swap pointers in O(1) time without moving 900 bytes in memory
    helius_cov15_t* temp = estimator->P;
    estimator->P = estimator->P_pred;
    estimator->P_pred = temp;
}

void helius_estimator_predict(helius_estimator_t *estimator, const helius_imu_t *imu_measurement, float dt)
{
    // Correct sensor with respective bias
    helius_vec3_t w_corr = {
        imu_measurement->gyro_x_rps - estimator->gyro_bias_rps.x,
        imu_measurement->gyro_y_rps - estimator->gyro_bias_rps.y,
        imu_measurement->gyro_z_rps - estimator->gyro_bias_rps.z};

    helius_vec3_t a_corr = {
        imu_measurement->accel_x_mps2 - estimator->accel_bias_mps2.x,
        imu_measurement->accel_y_mps2 - estimator->accel_bias_mps2.y,
        imu_measurement->accel_z_mps2 - estimator->accel_bias_mps2.z};

    predict_nominal_state(estimator, w_corr, a_corr, dt);
    predict_covariance(estimator, w_corr, a_corr, dt);
}