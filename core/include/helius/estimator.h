#ifndef HELIUS_ESTIMATOR_H
#define HELIUS_ESTIMATOR_H

#include <stdint.h>

#include "math/mat3.h"
#include "math/quat.h"
#include "math/vec3.h"
#include "types.h"

#define HELIUS_GRAVITY -9.80665f

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HELIUS_BLOCK_ATT = 0,  // Attitude error (3x3)
    HELIUS_BLOCK_POS = 1,  // Position error (3x3)
    HELIUS_BLOCK_VEL = 2,  // Velocity error (3x3)
    HELIUS_BLOCK_BG  = 3,  // Gyroscope bias error (3x3)
    HELIUS_BLOCK_BA  = 4,  // Accelerometer bias error (3x3)
    
    HELIUS_BLOCK_COUNT = 5 // Total 3x3 blocks (5 * 3 = 15 states)
} helius_state_block_t;

typedef struct {
    // ===========================
    // 16 NOMINAL STATE VECTOR
    // ===========================
    helius_quat_t attitude_rad;
    helius_vec3_t position_ned;
    helius_vec3_t velocity_mps;
    helius_vec3_t gyro_bias_rps;
    helius_vec3_t accel_bias_mps2;

    // ===========================
    // CONFIGURATION & BUFFERS
    // ===========================
    helius_noise_config_t noise;

    helius_vec3_t a_nav_prev;

    helius_cov15_t* P;      // Pointer to current active covariance
    helius_cov15_t* P_pred; // Pointer to next predicted covariance (scratchpad)

    helius_cov15_t P_buffer_a; // Concrete storage A in SRAM
    helius_cov15_t P_buffer_b; // Concrete storage B in SRAM

    uint8_t is_initialized;

} helius_estimator_t;

void helius_estimator_init(helius_estimator_t* estimator, const helius_noise_config_t* noise_cfg);
void helius_estimator_predict(
    helius_estimator_t* estimator,
    const helius_imu_t* imu_measurement,
    float dt
);


#ifdef __cplusplus
}
#endif

#endif