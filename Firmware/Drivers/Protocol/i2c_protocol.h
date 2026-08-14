/*
 * i2c_protocol.h
 *
 *  Created on: 13 de ago. de 2026
 *      Author: breno
 */

#ifndef DRIVERS_PROTOCOL_I2C_PROTOCOL_H_
#define DRIVERS_PROTOCOL_I2C_PROTOCOL_H_

#include <stm32f4xx_hal.h>

typedef enum
{
    I2C_PROTOCOL_OK = 0,
    I2C_PROTOCOL_ERROR,
    I2C_PROTOCOL_BUSY,
    I2C_PROTOCOL_TIMEOUT
} I2C_ProtocolStatus;

typedef struct
{
	I2C_HandleTypeDef* handle;
	uint32_t timeout_ms;
} I2C_Protocol;

I2C_ProtocolStatus i2c_protocol_init(
    I2C_Protocol *protocol,
    I2C_HandleTypeDef *handle,
    uint32_t timeout_ms
);

I2C_ProtocolStatus i2c_protocol_read(
    I2C_Protocol *protocol,
    uint16_t address,
    uint8_t reg,
    uint8_t *data,
    uint16_t size
);

I2C_ProtocolStatus i2c_protocol_write(
    I2C_Protocol *protocol,
    uint16_t address,
    uint8_t reg,
    const uint8_t *data,
    uint16_t size
);

#endif /* DRIVERS_PROTOCOL_I2C_PROTOCOL_H_ */
