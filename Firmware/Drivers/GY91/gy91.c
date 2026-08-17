#include "gy91.h"

static void gy91_mag_to_body(
    const vec3_t *mag_sensor,
    vec3_t *mag_body
)
{
    mag_body->x = mag_sensor->y;
    mag_body->y = mag_sensor->x;
    mag_body->z = -mag_sensor->z;
}

GY91_Status gy91_init(GY91_Driver *driver, I2C_Protocol *i2c, uint16_t mpu9250_address, uint16_t ak8963_address)
{
    if (driver == NULL || i2c == NULL)
    {
        return GY91_STATUS_INVALID_PARAM;
    }

    driver->i2c = i2c;

    // Initialize MPU9250
    if (mpu9250_init(&driver->mpu9250, i2c, mpu9250_address) != MPU9250_STATUS_OK)
    {
        return GY91_STATUS_ERROR;
    }

    // Disable MPU9250's I2C master mode to allow direct access to the AK8963 via the MPU9250's bypass mode
    if (mpu9250_set_i2c_master(&driver->mpu9250, false) != MPU9250_STATUS_OK)
    {
        return GY91_STATUS_ERROR;
    }

    if (mpu9250_set_bypass(&driver->mpu9250, true) != MPU9250_STATUS_OK)
    {
        return GY91_STATUS_ERROR;
    }

    // Initialize AK8963
    if (ak8963_init(&driver->ak8963, i2c, ak8963_address) != AK8963_STATUS_OK)
    {
        return GY91_STATUS_ERROR;
    }

    driver->initialized = true;

    return GY91_STATUS_OK;
}

GY91_Status gy91_read_mpu9250_data(GY91_Driver *driver)
{
    if (driver == NULL)
    {
        return GY91_STATUS_INVALID_PARAM;
    }

    if (!driver->initialized)
    {
        return GY91_STATUS_NOT_INITIALIZED;
    }

    if(mpu9250_read_data(&driver->mpu9250) != MPU9250_STATUS_OK)
    {
        return GY91_STATUS_ERROR;
    }

    driver->data.accel_mps2 = driver->mpu9250.data.accel_mps2;
    driver->data.gyro_rps = driver->mpu9250.data.gyro_rps;

    return GY91_STATUS_OK;
}

GY91_Status gy91_read_ak8963_data(GY91_Driver *driver)
{
    if (driver == NULL)
    {
        return GY91_STATUS_INVALID_PARAM;
    }

    if (!driver->initialized)
    {
        return GY91_STATUS_NOT_INITIALIZED;
    }

    if (ak8963_read_data(&driver->ak8963) != AK8963_STATUS_OK)
    {
        return GY91_STATUS_ERROR;
    }

    gy91_mag_to_body(&driver->ak8963.data.mag_ut, &driver->data.mag_ut);

    return GY91_STATUS_OK;
}
