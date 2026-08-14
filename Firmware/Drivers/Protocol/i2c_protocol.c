/*
 * i2c_protocol.c
 *
 *  Created on: 13 de ago. de 2026
 *      Author: breno
 */

#include "i2c_protocol.h"

static I2C_ProtocolStatus get_status(HAL_StatusTypeDef hal_status)
{
	switch (hal_status)
	    {
	        case HAL_OK:
	            return I2C_PROTOCOL_OK;

	        case HAL_BUSY:
	            return I2C_PROTOCOL_BUSY;

	        case HAL_TIMEOUT:
	            return I2C_PROTOCOL_TIMEOUT;

	        default:
	            return I2C_PROTOCOL_ERROR;
	    }
}

I2C_ProtocolStatus i2c_protocol_init(
    I2C_Protocol *protocol,
    I2C_HandleTypeDef *handle,
    uint32_t timeout_ms
)
{
    if (protocol == NULL || handle == NULL)
        return I2C_PROTOCOL_ERROR;

    protocol->handle = handle;
    protocol->timeout_ms = timeout_ms;

    return I2C_PROTOCOL_OK;
}

I2C_ProtocolStatus i2c_protocol_read(
    I2C_Protocol *protocol,
    uint16_t address,
    uint8_t reg,
    uint8_t *data,
    uint16_t size
)
{
    HAL_StatusTypeDef status;

    if (protocol == NULL || protocol->handle == NULL ||
        data == NULL || size == 0)
    {
        return I2C_PROTOCOL_ERROR;
    }

    status = HAL_I2C_Mem_Read(
        protocol->handle,
        address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        size,
        protocol->timeout_ms
    );

    return get_status(status);
}

I2C_ProtocolStatus i2c_protocol_write(
    I2C_Protocol *protocol,
    uint16_t address,
    uint8_t reg,
    const uint8_t *data,
    uint16_t size
)
{
    HAL_StatusTypeDef status;

    if (protocol == NULL || protocol->handle == NULL ||
        data == NULL || size == 0)
    {
        return I2C_PROTOCOL_ERROR;
    }

    status = HAL_I2C_Mem_Write(
        protocol->handle,
        address,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)data,
        size,
        protocol->timeout_ms
    );

    return get_status(status);
}
