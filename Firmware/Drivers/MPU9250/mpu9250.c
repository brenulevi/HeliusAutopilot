/*
 * mpu9250.c
 *
 *  Created on: 13 de ago. de 2026
 *      Author: breno
 */

#include "mpu9250.h"
#include "mpu9250_registers.h"

#define MPU9250_PWR_MGMT_1_CLKSEL_XGYRO  0x01

static MPU9250_Status mpu9250_get_accel_scale(
    MPU9250_AccelRange range,
    float *scale
)
{
    if (scale == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    switch (range)
    {
        case MPU9250_ACCEL_RANGE_2G:
            *scale = 16384.0f;
            return MPU9250_STATUS_OK;

        case MPU9250_ACCEL_RANGE_4G:
            *scale = 8192.0f;
            return MPU9250_STATUS_OK;

        case MPU9250_ACCEL_RANGE_8G:
            *scale = 4096.0f;
            return MPU9250_STATUS_OK;

        case MPU9250_ACCEL_RANGE_16G:
            *scale = 2048.0f;
            return MPU9250_STATUS_OK;

        default:
            return MPU9250_STATUS_INVALID_PARAM;
    }
}

static MPU9250_Status mpu9250_get_gyro_scale(
    MPU9250_GyroRange range,
    float *scale
)
{
    if (scale == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    switch (range)
    {
        case MPU9250_GYRO_RANGE_250DPS:
            *scale = 131.0f;
            return MPU9250_STATUS_OK;

        case MPU9250_GYRO_RANGE_500DPS:
            *scale = 65.5f;
            return MPU9250_STATUS_OK;

        case MPU9250_GYRO_RANGE_1000DPS:
            *scale = 32.8f;
            return MPU9250_STATUS_OK;

        case MPU9250_GYRO_RANGE_2000DPS:
            *scale = 16.4f;
            return MPU9250_STATUS_OK;

        default:
            return MPU9250_STATUS_INVALID_PARAM;
    }
}

static MPU9250_Status mpu9250_apply_accel_range(
    MPU9250_Driver *driver,
    MPU9250_AccelRange range
)
{
    float scale;

    if (mpu9250_get_accel_scale(range, &scale)
        != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)range << 3);

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_ACCEL_CONFIG,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    driver->config.accel_range = range;
    driver->config.accel_lsb_per_g = scale;

    return MPU9250_STATUS_OK;
}

static MPU9250_Status mpu9250_apply_gyro_range(
    MPU9250_Driver *driver,
    MPU9250_GyroRange range
)
{
    float scale;

    if (mpu9250_get_gyro_scale(range, &scale)
        != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)range << 3);

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_GYRO_CONFIG,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    driver->config.gyro_range = range;
    driver->config.gyro_lsb_per_dps = scale;

    return MPU9250_STATUS_OK;
}

static MPU9250_Status mpu9250_set_clock(
    MPU9250_Driver *driver
)
{
    if (mpu9250_write_register(
            driver,
            MPU9250_REG_PWR_MGMT_1,
            MPU9250_PWR_MGMT_1_CLKSEL_XGYRO) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    return MPU9250_STATUS_OK;
}

static MPU9250_Status mpu9250_configure(
    MPU9250_Driver *driver
)
{
    uint8_t value;

    /*
     * CONFIG
     *
     * DLPF_CFG:
     * 0 = 250 Hz
     * 1 = 184 Hz
     * 2 = 92 Hz
     * 3 = 41 Hz
     * ...
     */
    value = (uint8_t)driver->config.dlpf;

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_CONFIG,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    /*
     * Gyroscope configuration
     *
     * FS_SEL occupies bits 4:3.
     */
    value = ((uint8_t)driver->config.gyro_range << 3);

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_GYRO_CONFIG,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    /*
     * Accelerometer configuration
     *
     * AFS_SEL occupies bits 4:3.
     */
    value = ((uint8_t)driver->config.accel_range << 3);

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_ACCEL_CONFIG,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    /*
     * Accelerometer DLPF enabled.
     */
    value = (uint8_t)driver->config.dlpf;

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_ACCEL_CONFIG_2,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    /*
     * Sample rate.
     *
     * With gyro internal rate = 1 kHz:
     *
     * SampleRate = 1000 / (1 + SMPLRT_DIV)
     *
     * For 1 kHz:
     *
     * SMPLRT_DIV = 0
     */
    value = 0;

    if (mpu9250_write_register(
            driver,
            MPU9250_REG_SMPLRT_DIV,
            value) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    return MPU9250_STATUS_OK;
}

MPU9250_Status mpu9250_init(
    MPU9250_Driver *driver,
    I2C_Protocol *i2c,
    uint16_t address
)
{
    uint8_t who_am_i;

    if (driver == NULL || i2c == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    driver->i2c = i2c;
    driver->address = address;
    driver->initialized = false;

    /*
     * Default configuration
     */
    if (mpu9250_apply_accel_range(
            driver,
            MPU9250_ACCEL_RANGE_2G) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    if (mpu9250_apply_gyro_range(
			driver,
			MPU9250_GYRO_RANGE_250DPS) != MPU9250_STATUS_OK)
	{
		return MPU9250_STATUS_ERROR;
	}

    driver->config.dlpf =
        MPU9250_DLPF_41HZ;

    driver->config.sample_rate_hz = 1000;

    /*
     * Check device identity.
     */
    if (mpu9250_read_register(
            driver,
            MPU9250_REG_WHO_AM_I,
            &who_am_i) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    if (who_am_i != MPU9250_WHO_AM_I_VALUE)
    {
        return MPU9250_STATUS_NOT_FOUND;
    }

    /*
     * Reset device.
     */
    if (mpu9250_write_register(
            driver,
            MPU9250_REG_PWR_MGMT_1,
            MPU9250_PWR_MGMT_1_DEVICE_RESET) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    HAL_Delay(100);

    /*
     * Select gyro X PLL clock.
     */
    if (mpu9250_set_clock(driver) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    /*
     * Configure sensor.
     */
    if (mpu9250_configure(driver) != MPU9250_STATUS_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    driver->initialized = true;

    return MPU9250_STATUS_OK;
}

MPU9250_Status mpu9250_set_accel_range(
    MPU9250_Driver *driver,
    MPU9250_AccelRange range
)
{
    if (driver == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    if (!driver->initialized)
    {
        return MPU9250_STATUS_NOT_INITIALIZED;
    }

    return mpu9250_apply_accel_range(driver, range);
}

MPU9250_Status mpu9250_set_gyro_range(
    MPU9250_Driver *driver,
    MPU9250_GyroRange range
)
{
	if (driver == NULL)
	{
		return MPU9250_STATUS_INVALID_PARAM;
	}

	if (!driver->initialized)
	{
		return MPU9250_STATUS_NOT_INITIALIZED;
	}

	return mpu9250_apply_gyro_range(driver, range);
}

MPU9250_Status mpu9250_read_data(
    MPU9250_Driver *driver
)
{
    uint8_t buffer[14];

    if (driver == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    if (!driver->initialized)
    {
        return MPU9250_STATUS_NOT_INITIALIZED;
    }

    if (i2c_protocol_read(
            driver->i2c,
            driver->address,
            MPU9250_REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)) != I2C_PROTOCOL_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    /*
     * ACCEL
     */

    int16_t accel_x =
        (int16_t)((buffer[0] << 8) | buffer[1]);

    int16_t accel_y =
        (int16_t)((buffer[2] << 8) | buffer[3]);

    int16_t accel_z =
        (int16_t)((buffer[4] << 8) | buffer[5]);

	 const float gravity = 9.80665f;

	 driver->data.accel_mps2.x =
		 ((float)accel_x / driver->config.accel_lsb_per_g) * gravity;

	 driver->data.accel_mps2.y =
		 ((float)accel_y / driver->config.accel_lsb_per_g) * gravity;

	 driver->data.accel_mps2.z =
		 ((float)accel_z / driver->config.accel_lsb_per_g) * gravity;

    /*
     * TEMPERATURE
     */

    int16_t temperature =
        (int16_t)((buffer[6] << 8) | buffer[7]);

    driver->data.temperature_c =
                ((float)temperature / 333.87f) + 21.0f;

    /*
     * GYRO
     */

    int16_t gyro_x =
        (int16_t)((buffer[8] << 8) | buffer[9]);

    int16_t gyro_y =
        (int16_t)((buffer[10] << 8) | buffer[11]);

    int16_t gyro_z =
        (int16_t)((buffer[12] << 8) | buffer[13]);

   const float deg_to_rad = 0.017453292519943295f;

   driver->data.gyro_rps.x =
	   ((float)gyro_x / driver->config.gyro_lsb_per_dps) * deg_to_rad;

   driver->data.gyro_rps.y =
	   ((float)gyro_y / driver->config.gyro_lsb_per_dps) * deg_to_rad;

   driver->data.gyro_rps.z =
	   ((float)gyro_z / driver->config.gyro_lsb_per_dps) * deg_to_rad;

   return MPU9250_STATUS_OK;
}

MPU9250_Status mpu9250_read_register(
    MPU9250_Driver *driver,
    uint8_t reg,
    uint8_t *data
)
{
    if (driver == NULL ||
        driver->i2c == NULL ||
        data == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    I2C_ProtocolStatus status;

    status = i2c_protocol_read(
        driver->i2c,
        driver->address,
        reg,
        data,
        1
    );

    if (status != I2C_PROTOCOL_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    return MPU9250_STATUS_OK;
}

MPU9250_Status mpu9250_write_register(
    MPU9250_Driver *driver,
    uint8_t reg,
    uint8_t data
)
{
    if (driver == NULL ||
        driver->i2c == NULL)
    {
        return MPU9250_STATUS_INVALID_PARAM;
    }

    I2C_ProtocolStatus status;

    status = i2c_protocol_write(
        driver->i2c,
        driver->address,
        reg,
        &data,
        1
    );

    if (status != I2C_PROTOCOL_OK)
    {
        return MPU9250_STATUS_ERROR;
    }

    return MPU9250_STATUS_OK;
}
