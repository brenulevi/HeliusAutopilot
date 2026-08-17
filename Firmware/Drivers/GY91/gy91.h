#ifndef DRIVERS_GY91_H_
#define DRIVERS_GY91_H_

#include <stdint.h>
#include <stdbool.h>

#include "i2c_protocol.h"
#include "mpu9250.h"
#include "ak8963.h"

typedef enum
{
    GY91_STATUS_OK = 0,
    GY91_STATUS_ERROR,
    GY91_STATUS_INVALID_PARAM,
    GY91_STATUS_NOT_INITIALIZED,
    GY91_STATUS_NOT_FOUND
} GY91_Status;

typedef struct
{
    vec3_t accel_mps2;
    vec3_t gyro_rps;
    vec3_t mag_ut;
} GY91_Data;

typedef struct
{
    I2C_Protocol *i2c;

    MPU9250_Driver mpu9250;
    AK8963_Driver ak8963;

    GY91_Data data;

    bool initialized;
} GY91_Driver;

GY91_Status gy91_init(GY91_Driver *driver, I2C_Protocol *i2c, uint16_t mpu9250_address, uint16_t ak8963_address);
GY91_Status gy91_read_mpu9250_data(GY91_Driver *driver);
GY91_Status gy91_read_ak8963_data(GY91_Driver *driver);

#endif // DRIVERS_GY91_H_