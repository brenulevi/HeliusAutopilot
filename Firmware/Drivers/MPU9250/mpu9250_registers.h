/*
 * mpu9250_registers.h
 *
 *  Created on: 13 de ago. de 2026
 *      Author: breno
 */

#ifndef DRIVERS_MPU9250_MPU9250_REGISTERS_H_
#define DRIVERS_MPU9250_MPU9250_REGISTERS_H_

/* Device identification */
#define MPU9250_REG_WHO_AM_I             0x75
#define MPU9250_WHO_AM_I_VALUE           0x71

/* Power management */
#define MPU9250_REG_PWR_MGMT_1           0x6B
#define MPU9250_REG_PWR_MGMT_2           0x6C

/* User control */
#define MPU9250_REG_USER_CTRL            0x6A

/* Configuration */
#define MPU9250_REG_CONFIG               0x1A
#define MPU9250_REG_SMPLRT_DIV           0x19

/* Gyroscope */
#define MPU9250_REG_GYRO_CONFIG          0x1B

/* Accelerometer */
#define MPU9250_REG_ACCEL_CONFIG         0x1C
#define MPU9250_REG_ACCEL_CONFIG_2       0x1D

/* Accelerometer output */
#define MPU9250_REG_ACCEL_XOUT_H         0x3B
#define MPU9250_REG_ACCEL_XOUT_L         0x3C
#define MPU9250_REG_ACCEL_YOUT_H         0x3D
#define MPU9250_REG_ACCEL_YOUT_L         0x3E
#define MPU9250_REG_ACCEL_ZOUT_H         0x3F
#define MPU9250_REG_ACCEL_ZOUT_L         0x40

/* Temperature output */
#define MPU9250_REG_TEMP_OUT_H           0x41
#define MPU9250_REG_TEMP_OUT_L           0x42

/* Gyroscope output */
#define MPU9250_REG_GYRO_XOUT_H          0x43
#define MPU9250_REG_GYRO_XOUT_L          0x44
#define MPU9250_REG_GYRO_YOUT_H          0x45
#define MPU9250_REG_GYRO_YOUT_L          0x46
#define MPU9250_REG_GYRO_ZOUT_H          0x47
#define MPU9250_REG_GYRO_ZOUT_L          0x48

/* Signal path reset */
#define MPU9250_REG_SIGNAL_PATH_RESET    0x68

/* Device reset */
#define MPU9250_PWR_MGMT_1_DEVICE_RESET  0x80

#endif /* DRIVERS_MPU9250_MPU9250_REGISTERS_H_ */
