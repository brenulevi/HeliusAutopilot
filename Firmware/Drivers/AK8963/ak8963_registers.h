#ifndef AK8963_REGISTERS_H_
#define AK8963_REGISTERS_H_

#define AK8963_REG_WHO_AM_I     0x00
#define AK8963_WHO_AM_I_VALUE   0x48

#define AK8963_REG_CNTL1        0x0A
#define AK8963_REG_CNTL2        0x0B

#define AK8963_REG_ASAX         0x10
#define AK8963_REG_ASAY         0x11
#define AK8963_REG_ASAZ         0x12

#define AK8963_REG_ST1          0x02
#define AK8963_REG_HXL          0x03
#define AK8963_REG_HXH          0x04
#define AK8963_REG_HYL          0x05
#define AK8963_REG_HYH          0x06
#define AK8963_REG_HZL          0x07
#define AK8963_REG_ST2          0x09

#define AK8963_ST1_DRDY         (1U << 0)
#define AK8963_ST2_HOFL         (1U << 3)

#endif // AK8963_REGISTERS_H_