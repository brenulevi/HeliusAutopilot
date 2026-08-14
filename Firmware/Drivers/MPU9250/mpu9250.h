/*
 * mpu9250.h
 *
 *  Created on: 13 de ago. de 2026
 *      Author: breno
 */

#ifndef DRIVERS_MPU9250_MPU9250_H_
#define DRIVERS_MPU9250_MPU9250_H_

#include <stdint.h>
#include <stdbool.h>

#include "i2c_protocol.h"
#include "mpu9250_registers.h"

typedef enum
{
    MPU9250_STATUS_OK = 0,
    MPU9250_STATUS_ERROR,
    MPU9250_STATUS_INVALID_PARAM,
    MPU9250_STATUS_NOT_INITIALIZED,
    MPU9250_STATUS_NOT_FOUND
} MPU9250_Status;

typedef enum
{
    MPU9250_ACCEL_RANGE_2G = 0,
    MPU9250_ACCEL_RANGE_4G = 1,
    MPU9250_ACCEL_RANGE_8G = 2,
    MPU9250_ACCEL_RANGE_16G = 3

} MPU9250_AccelRange;

typedef enum
{
    MPU9250_GYRO_RANGE_250DPS = 0,
    MPU9250_GYRO_RANGE_500DPS = 1,
    MPU9250_GYRO_RANGE_1000DPS = 2,
    MPU9250_GYRO_RANGE_2000DPS = 3

} MPU9250_GyroRange;

typedef enum
{
    MPU9250_DLPF_250HZ = 0,
    MPU9250_DLPF_184HZ = 1,
    MPU9250_DLPF_92HZ = 2,
    MPU9250_DLPF_41HZ = 3,
    MPU9250_DLPF_20HZ = 4,
    MPU9250_DLPF_10HZ = 5,
    MPU9250_DLPF_5HZ = 6

} MPU9250_DLPF;

typedef struct
{
    MPU9250_AccelRange accel_range;
    MPU9250_GyroRange gyro_range;

    MPU9250_DLPF dlpf;

    uint16_t sample_rate_hz;

    float accel_lsb_per_g;
	float gyro_lsb_per_dps;

} MPU9250_Config;

typedef struct
{
    float x;
    float y;
    float z;

} MPU9250_Vector3;

typedef struct
{
    MPU9250_Vector3 accel_mps2;
    MPU9250_Vector3 gyro_rps;

    float temperature_c;

} MPU9250_Data;

typedef struct
{
    I2C_Protocol *i2c;

    uint16_t address;

    MPU9250_Config config;

    MPU9250_Data data;

    bool initialized;
} MPU9250_Driver;

MPU9250_Status mpu9250_init(
    MPU9250_Driver *driver,
    I2C_Protocol *i2c,
    uint16_t address
);

MPU9250_Status mpu9250_set_accel_range(
    MPU9250_Driver *driver,
    MPU9250_AccelRange range
);

MPU9250_Status mpu9250_set_gyro_range(
    MPU9250_Driver *driver,
    MPU9250_GyroRange range
);

MPU9250_Status mpu9250_read_data(
    MPU9250_Driver *driver
);

MPU9250_Status mpu9250_read_register(
    MPU9250_Driver *driver,
    uint8_t reg,
    uint8_t *data
);

MPU9250_Status mpu9250_write_register(
    MPU9250_Driver *driver,
    uint8_t reg,
    uint8_t data
);

#endif /* DRIVERS_MPU9250_MPU9250_H_ */
