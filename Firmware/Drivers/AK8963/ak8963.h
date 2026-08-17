#ifndef DRIVERS_AK8963_H_
#define DRIVERS_AK8963_H_

#include <stdint.h>
#include <stdbool.h>
#include <helius/math/vec3.h>

#include "i2c_protocol.h"

typedef enum
{
    AK8963_STATUS_OK = 0,
    AK8963_STATUS_ERROR,
    AK8963_STATUS_INVALID_PARAM,
    AK8963_STATUS_NOT_INITIALIZED,
    AK8963_STATUS_NOT_FOUND,
    AK8963_STATUS_NOT_READY,
    AK8963_STATUS_OVERFLOW
} AK8963_Status;

typedef enum
{
    AK8963_RESOLUTION_14BIT = 0x00,
    AK8963_RESOLUTION_16BIT = 0x10
} AK8963_Resolution;

typedef enum
{
    AK8963_MODE_POWER_DOWN = 0x00,
    AK8963_MODE_SINGLE = 0x01,
    AK8963_MODE_CONTINUOUS1 = 0x02, // 8 Hz
    AK8963_MODE_CONTINUOUS_2 = 0x06, // 100 Hz
    AK8963_MODE_FUSE_ROM_ACCESS = 0x0F
} AK8963_Mode;

typedef struct
{
    AK8963_Resolution resolution;
    AK8963_Mode mode;

    float ut_per_lsb; // microtesla per LSB

    vec3_t asa; // Sensitivity adjustment values from the fuse ROM
} AK8963_Config;

typedef struct
{
    vec3_t mag_ut; // Magnetic field in microtesla
} AK8963_Data;

typedef struct
{
    I2C_Protocol *i2c;

    uint16_t address;

    AK8963_Config config;

    AK8963_Data data;

    bool initialized;
} AK8963_Driver;

AK8963_Status ak8963_init(
    AK8963_Driver *driver,
    I2C_Protocol *i2c,
    uint16_t address
);

AK8963_Status ak8963_read_data(
    AK8963_Driver *driver
);

AK8963_Status ak8963_read_register(
    AK8963_Driver *driver,
    uint8_t reg,
    uint8_t *data
);

AK8963_Status ak8963_read_registers(
    AK8963_Driver *driver,
    uint8_t reg,
    uint8_t *data,
    size_t length
);

AK8963_Status ak8963_write_register(
    AK8963_Driver *driver,
    uint8_t reg,
    uint8_t data
);

#endif // DRIVERS_AK8963_H_