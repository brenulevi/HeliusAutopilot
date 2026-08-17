#include "calibration.h"

#include <stddef.h>
#include <stdint.h>
#include <math.h>

#define MAG_HARD_IRON_MAX_UT 200.0f
#define MAG_SOFT_IRON_MAX 5.0f

calibration_status_t calibration_apply_mag(const mag_calibration_t *calibration, const vec3_t *mag_raw_ut, vec3_t *mag_calibrated_ut)
{
    if (!calibration || !mag_raw_ut || !mag_calibrated_ut)
    {
        return CALIBRATION_STATUS_INVALID_ARGUMENT;
    }

    // Apply hard iron correction
    vec3_t mag_hard_iron_corrected;
    mag_hard_iron_corrected.x = mag_raw_ut->x - calibration->hard_iron_ut.x;
    mag_hard_iron_corrected.y = mag_raw_ut->y - calibration->hard_iron_ut.y;
    mag_hard_iron_corrected.z = mag_raw_ut->z - calibration->hard_iron_ut.z;

    // Apply soft iron correction
    mag_calibrated_ut->x = calibration->soft_iron.data[0][0] * mag_hard_iron_corrected.x +
                           calibration->soft_iron.data[0][1] * mag_hard_iron_corrected.y +
                           calibration->soft_iron.data[0][2] * mag_hard_iron_corrected.z;

    mag_calibrated_ut->y = calibration->soft_iron.data[1][0] * mag_hard_iron_corrected.x +
                           calibration->soft_iron.data[1][1] * mag_hard_iron_corrected.y +
                           calibration->soft_iron.data[1][2] * mag_hard_iron_corrected.z;

    mag_calibrated_ut->z = calibration->soft_iron.data[2][0] * mag_hard_iron_corrected.x +
                           calibration->soft_iron.data[2][1] * mag_hard_iron_corrected.y +
                           calibration->soft_iron.data[2][2] * mag_hard_iron_corrected.z;

    return CALIBRATION_STATUS_OK;
}

calibration_status_t calibration_validate_mag(const mag_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return CALIBRATION_STATUS_INVALID_ARGUMENT;
    }

    if (!isfinite(calibration->hard_iron_ut.x) ||
        !isfinite(calibration->hard_iron_ut.y) ||
        !isfinite(calibration->hard_iron_ut.z))
    {
        return CALIBRATION_STATUS_INVALID_DATA;
    }

    if (fabsf(calibration->hard_iron_ut.x) > MAG_HARD_IRON_MAX_UT ||
        fabsf(calibration->hard_iron_ut.y) > MAG_HARD_IRON_MAX_UT ||
        fabsf(calibration->hard_iron_ut.z) > MAG_HARD_IRON_MAX_UT)
    {
        return CALIBRATION_STATUS_OUT_OF_RANGE;
    }

    for (uint32_t i = 0; i < 3; i++)
    {
        for (uint32_t j = 0; j < 3; j++)
        {
            if (!isfinite(calibration->soft_iron.data[i][j]))
            {
                return CALIBRATION_STATUS_INVALID_DATA;
            }

            if (fabsf(calibration->soft_iron.data[i][j] > MAG_SOFT_IRON_MAX))
            {
                return CALIBRATION_STATUS_OUT_OF_RANGE;
            }
        }
    }

    float det = mat3_det(&calibration->soft_iron);
    if (fabsf(det) < 1e-6f)
    {
        return CALIBRATION_STATUS_INVALID_MATRIX;
    }

    return CALIBRATION_STATUS_OK;
}

calibration_status_t calibration_mag_identity(mag_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return CALIBRATION_STATUS_INVALID_ARGUMENT;
    }

    calibration->hard_iron_ut.x = 0.0f;
    calibration->hard_iron_ut.y = 0.0f;
    calibration->hard_iron_ut.z = 0.0f;

    calibration->soft_iron.data[0][0] = 1.0f;
    calibration->soft_iron.data[0][1] = 0.0f;
    calibration->soft_iron.data[0][2] = 0.0f;

    calibration->soft_iron.data[1][0] = 0.0f;
    calibration->soft_iron.data[1][1] = 1.0f;
    calibration->soft_iron.data[1][2] = 0.0f;

    calibration->soft_iron.data[2][0] = 0.0f;
    calibration->soft_iron.data[2][1] = 0.0f;
    calibration->soft_iron.data[2][2] = 1.0f;

    return CALIBRATION_STATUS_OK;
}
