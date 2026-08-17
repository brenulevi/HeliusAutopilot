#ifndef HELIUS_CORE_H_
#define HELIUS_CORE_H_

#include <stdint.h>

#include "estimator/ahrs.h"
#include "calibration.h"
#include "command.h"

typedef enum
{
    CORE_STATUS_OK = 0,

    CORE_STATUS_INVALID_ARGUMENT,
    CORE_STATUS_INVALID_COMMAND,
    CORE_STATUS_INVALID_STATE,
    CORE_STATUS_INVALID_CALIBRATION,

    CORE_STATUS_AHRS_INIT_FAILED,
    CORE_STATUS_AHRS_PREDICT_FAILED,
    CORE_STATUS_AHRS_UPDATE_ACCEL_FAILED,
    CORE_STATUS_AHRS_UPDATE_MAG_FAILED,

    CORE_STATUS_CALIBRATION_FAILED,
    CORE_STATUS_CALIBRATION_ACTIVE
} core_status_t;

typedef enum
{
    CORE_MODE_NORMAL = 0,
    CORE_MODE_MAG_CALIBRATION,

    CORE_MODE_INVALID
} core_mode_t;

typedef struct
{
    ahrs_config_t ahrs_config;
} core_config_t;

typedef struct
{
    core_mode_t mode;

    ahrs_t ahrs;

    core_config_t config;

    mag_calibration_t mag_calibration;
    uint32_t mag_cal_sample_count;
} core_t;

core_status_t core_init(core_t *core, const core_config_t *config, const mag_calibration_t* mag_calibration);
core_status_t core_command(core_t *core, core_command_t command);
core_status_t core_update_gyro(
    core_t *core,
    const vec3_t *gyro_rps,
    float dt);
core_status_t core_update_accel(
    core_t *core,
    const vec3_t *accel_mps2);
core_status_t core_update_mag(core_t *core, const vec3_t *mag_ut);
core_mode_t core_get_mode(const core_t *core);
core_status_t core_set_mag_calibration(
    core_t* core,
    const mag_calibration_t* calibration
);

#endif // HELIUS_CORE_H_