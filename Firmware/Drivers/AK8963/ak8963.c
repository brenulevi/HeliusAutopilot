#include "ak8963.h"
#include "ak8963_registers.h"

static AK8963_Status ak8963_get_scale(AK8963_Resolution resolution, float *scale)
{
    if (scale == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    switch (resolution)
    {
    case AK8963_RESOLUTION_14BIT:
        *scale = 0.6f; // 0.6 uT per LSB
        return AK8963_STATUS_OK;

    case AK8963_RESOLUTION_16BIT:
        *scale = 0.15f; // 0.15 uT per LSB
        return AK8963_STATUS_OK;

    default:
        return AK8963_STATUS_INVALID_PARAM;
    }
}

static AK8963_Status ak8963_get_factory_calibration(AK8963_Driver *driver)
{
    if (driver == NULL || driver->i2c == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    // Set to Fuse ROM access mode
    if (ak8963_write_register(driver, AK8963_REG_CNTL1, AK8963_MODE_FUSE_ROM_ACCESS) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    // Read sensitivity adjustment values from the fuse ROM
    uint8_t asa[3];
    
    // ASAX = 0x10, ASAY = 0x11, ASAZ = 0x12
    if(ak8963_read_registers(driver, AK8963_REG_ASAX, asa, 3) != AK8963_STATUS_OK)
    {
        // Return to power-down mode before returning error
        (void)ak8963_write_register(
            driver,
            AK8963_REG_CNTL1,
            AK8963_MODE_POWER_DOWN);

        return AK8963_STATUS_ERROR;
    }

    // Convert the sensitivity adjustment values to a scale factor
    driver->config.asa.x = ((float)(asa[0] - 128) / 256.0f) + 1.0f;
    driver->config.asa.y = ((float)(asa[1] - 128) / 256.0f) + 1.0f;
    driver->config.asa.z = ((float)(asa[2] - 128) / 256.0f) + 1.0f;

    // Return to power-down mode
    if (ak8963_write_register(driver, AK8963_REG_CNTL1, AK8963_MODE_POWER_DOWN) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    return AK8963_STATUS_OK;
}

static AK8963_Status ak8963_configure(
    AK8963_Driver *driver,
    AK8963_Resolution resolution,
    AK8963_Mode mode)
{
    if (driver == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    float scale;

    if (ak8963_get_scale(resolution, &scale) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    uint8_t value = ((uint8_t)resolution | (uint8_t)mode);

    if (ak8963_write_register(
            driver,
            AK8963_REG_CNTL1,
            value) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    driver->config.resolution = resolution;
    driver->config.mode = mode;
    driver->config.ut_per_lsb = scale;

    return AK8963_STATUS_OK;
}

AK8963_Status ak8963_init(
    AK8963_Driver *driver,
    I2C_Protocol *i2c,
    uint16_t address)
{
    if (driver == NULL || i2c == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    driver->i2c = i2c;
    driver->address = address;
    driver->initialized = false;

    // Check WHO_AM_I register
    uint8_t who_am_i;

    if (ak8963_read_register(driver, AK8963_REG_WHO_AM_I, &who_am_i) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    if (who_am_i != AK8963_WHO_AM_I_VALUE)
    {
        return AK8963_STATUS_NOT_FOUND;
    }

    // Read factory calibration values from the fuse ROM
    if (ak8963_get_factory_calibration(driver) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    // Set default configuration
    // Default to 16-bit resolution and continuous measurement mode 2 (100 Hz)
    if (ak8963_configure(driver,
                         AK8963_RESOLUTION_16BIT,
                         AK8963_MODE_CONTINUOUS_2) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    driver->initialized = true;

    return AK8963_STATUS_OK;
}

AK8963_Status ak8963_read_data(AK8963_Driver *driver)
{
    if(driver == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    if(!driver->initialized)
    {
        return AK8963_STATUS_NOT_INITIALIZED;
    }

    uint8_t st1;

    if(ak8963_read_register(driver, AK8963_REG_ST1, &st1) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    // Check if data is ready
    if(!(st1 & AK8963_ST1_DRDY))
    {
        return AK8963_STATUS_NOT_READY;
    }

    // Read magnetometer data (6 bytes: HXL, HXH, HYL, HYH, HZL, HZH)
    uint8_t raw[7];

    if(ak8963_read_registers(driver, AK8963_REG_HXL, raw, sizeof(raw)) != AK8963_STATUS_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    const uint8_t st2 = raw[6];

    // Check for overflow
    if(st2 & AK8963_ST2_HOFL)
    {
        return AK8963_STATUS_OVERFLOW;
    }

    // Data is little-endian
    int16_t raw_x = (int16_t)((uint16_t)raw[1] << 8 | (uint16_t)raw[0]);
    int16_t raw_y = (int16_t)((uint16_t)raw[3] << 8 | (uint16_t)raw[2]);
    int16_t raw_z = (int16_t)((uint16_t)raw[5] << 8 | (uint16_t)raw[4]);

    // Convert raw data to uT
    // Apply factory calibration and scale factor

    const float scale = driver->config.ut_per_lsb;

    driver->data.mag_ut.x = (float)raw_x * scale * driver->config.asa.x;
    driver->data.mag_ut.y = (float)raw_y * scale * driver->config.asa.y;
    driver->data.mag_ut.z = (float)raw_z * scale * driver->config.asa.z;

    return AK8963_STATUS_OK;
}

AK8963_Status ak8963_read_register(AK8963_Driver *driver, uint8_t reg, uint8_t *data)
{
    if (driver == NULL || driver->i2c == NULL || data == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    I2C_ProtocolStatus status = i2c_protocol_read(
        driver->i2c,
        driver->address,
        reg,
        data,
        1);

    if (status != I2C_PROTOCOL_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    return AK8963_STATUS_OK;
}

AK8963_Status ak8963_read_registers(AK8963_Driver *driver, uint8_t reg, uint8_t *data, size_t length)
{
    if (driver == NULL || driver->i2c == NULL || data == NULL || length == 0)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    I2C_ProtocolStatus status = i2c_protocol_read(
        driver->i2c,
        driver->address,
        reg,
        data,
        length);

    if (status != I2C_PROTOCOL_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    return AK8963_STATUS_OK;
}

AK8963_Status ak8963_write_register(AK8963_Driver *driver, uint8_t reg, uint8_t data)
{
    if (driver == NULL || driver->i2c == NULL)
    {
        return AK8963_STATUS_INVALID_PARAM;
    }

    I2C_ProtocolStatus status = i2c_protocol_write(
        driver->i2c,
        driver->address,
        reg,
        &data,
        1);

    if (status != I2C_PROTOCOL_OK)
    {
        return AK8963_STATUS_ERROR;
    }

    return AK8963_STATUS_OK;
}