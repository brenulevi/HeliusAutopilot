#include "core.h"

#include <stddef.h>

static core_status_t core_start_mag_calibration(core_t *core)
{
    if (core->mode != CORE_MODE_NORMAL)
    {
        return CORE_STATUS_INVALID_STATE;
    }

    core->mode = CORE_MODE_MAG_CALIBRATION;
    core->mag_cal_sample_count = 0U;

    return CORE_STATUS_OK;
}

static core_status_t core_stop_mag_calibration(core_t *core)
{
    if (core->mode != CORE_MODE_MAG_CALIBRATION)
    {
        return CORE_STATUS_INVALID_STATE;
    }

    core->mode = CORE_MODE_NORMAL;

    return CORE_STATUS_OK;
}

core_status_t core_init(
    core_t *core,
    const core_config_t *config,
    const mag_calibration_t* mag_calibration)
{
    if (core == NULL || config == NULL)
    {
        return CORE_STATUS_INVALID_ARGUMENT;
    }

    core->mode = CORE_MODE_NORMAL;

    core->config = *config;

    core->mag_calibration = *mag_calibration;
    core->mag_cal_sample_count = 0U;

    ahrs_status_t status = ahrs_init(
        &core->ahrs,
        &core->config.ahrs_config);

    if (status != AHRS_STATUS_OK)
    {
        return CORE_STATUS_AHRS_INIT_FAILED;
    }

    return CORE_STATUS_OK;
}

core_status_t core_command(core_t *core, core_command_t command)
{
    if (core == NULL)
    {
        return CORE_STATUS_INVALID_ARGUMENT;
    }

    switch (command)
    {
    case CORE_CMD_MAG_CAL_START:
        return core_start_mag_calibration(core);

    case CORE_CMD_MAG_CAL_STOP:
        return core_stop_mag_calibration(core);

    case CORE_CMD_NONE:
    default:
        return CORE_STATUS_INVALID_COMMAND;
    }
}

core_status_t core_update_gyro(
    core_t *core,
    const vec3_t *gyro_rps,
    float dt)
{
    if (core == NULL || gyro_rps == NULL || dt <= 0.0f)
    {
        return CORE_STATUS_INVALID_ARGUMENT;
    }

    ahrs_status_t status = ahrs_predict(
        &core->ahrs,
        gyro_rps,
        dt);

    if (status != AHRS_STATUS_OK)
    {
        return CORE_STATUS_AHRS_PREDICT_FAILED;
    }

    return CORE_STATUS_OK;
}

core_status_t core_update_accel(core_t *core, const vec3_t *accel_mps2)
{
    if (core == NULL || accel_mps2 == NULL)
    {
        return CORE_STATUS_INVALID_ARGUMENT;
    }

    ahrs_status_t status = ahrs_update_accel(
        &core->ahrs,
        accel_mps2);

    if (status != AHRS_STATUS_OK)
    {
        return CORE_STATUS_AHRS_UPDATE_ACCEL_FAILED;
    }

    return CORE_STATUS_OK;
}

core_status_t core_update_mag(core_t *core, const vec3_t *mag_ut)
{
    if (core == NULL || mag_ut == NULL)
    {
        return CORE_STATUS_INVALID_ARGUMENT;
    }

    if (core->mode == CORE_MODE_MAG_CALIBRATION)
    {
        return CORE_STATUS_CALIBRATION_ACTIVE;
    }

    vec3_t corrected_mag;

    if (calibration_apply_mag(
        &core->mag_calibration,
        mag_ut,
        &corrected_mag) != CALIBRATION_STATUS_OK)
    {
        return CORE_STATUS_CALIBRATION_FAILED;
    }

    ahrs_status_t status = ahrs_update_mag(
        &core->ahrs,
        &corrected_mag);

    if (status != AHRS_STATUS_OK)
    {
        return CORE_STATUS_AHRS_UPDATE_MAG_FAILED;
    }

    return CORE_STATUS_OK;
}

core_mode_t core_get_mode(const core_t *core)
{
    if(core == NULL)
    {
        return CORE_MODE_INVALID; // Default to normal mode if core is NULL
    }

    return core->mode;
}

core_status_t core_set_mag_calibration(core_t *core, const mag_calibration_t *calibration)
{
    if(core == NULL || calibration == NULL)
    {
        return CORE_STATUS_INVALID_ARGUMENT;
    }

    if(calibration_validate_mag(calibration) != CALIBRATION_STATUS_OK)
    {
        return CORE_STATUS_INVALID_CALIBRATION;
    }

    core->mag_calibration = *calibration;

    return CORE_STATUS_OK;
}
