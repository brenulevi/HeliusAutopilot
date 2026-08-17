#ifndef HELIUS_CALIBRATION_H_
#define HELIUS_CALIBRATION_H_

#include "math/vec3.h"
#include "math/mat3.h"

typedef enum
{
    CALIBRATION_STATUS_OK = 0,
    CALIBRATION_STATUS_INVALID_ARGUMENT,
    CALIBRATION_STATUS_INVALID_DATA,
    CALIBRATION_STATUS_INVALID_MATRIX,

    CALIBRATION_STATUS_OUT_OF_RANGE,

    CALIBRATION_STATUS_ERROR
} calibration_status_t;

typedef struct
{
    vec3_t hard_iron_ut;
    mat3_t soft_iron;
} mag_calibration_t;

calibration_status_t calibration_apply_mag(
    const mag_calibration_t *calibration,
    const vec3_t *mag_raw_ut,
    vec3_t *mag_calibrated_ut
);
calibration_status_t calibration_validate_mag(const mag_calibration_t* calibration);
calibration_status_t calibration_mag_identity(mag_calibration_t* calibration);

#endif // HELIUS_CALIBRATION_H_