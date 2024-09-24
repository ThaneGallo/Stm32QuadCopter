#ifndef ICM_20948_H
#define ICM_20948_H

// comm protocols
#define ICM_SPI 0x00
#define ICM_I2C 0x01

// Gyro setup
// full scale
#define GYRO_FS_250 0x00
#define GYRO_FS_500 0x02
#define GYRO_FS_1000 0x04
#define GYRO_FS_2000 0x05

// how many samples to avg
#define GYRO_AVG_CFG_1 0x00
#define GYRO_AVG_CFG_2 0x01
#define GYRO_AVG_CFG_4 0x02
#define GYRO_AVG_CFG_8 0x03
#define GYRO_AVG_CFG_16 0x04
#define GYRO_AVG_CFG_32 0x05
#define GYRO_AVG_CFG_64 0x06
#define GYRO_AVG_CFG_128 0x07

// ACCEL setup
// full scale
#define ACCEL_FS_2G 0x00
#define ACCEL_FS_4G 0x02
#define ACCEL_FS_8G 0x04
#define ACCEL_FS_16G 0x05

// how many samples to avg
#define ACCEL_AVG_CFG_4 0x00
#define ACCEL_AVG_CFG_8 0x01
#define ACCEL_AVG_CFG_16 0x02
#define ACCEL_AVG_CFG_32 0x03


struct GYRO;
struct ACCEL;
struct ICM_20948;

struct GYRO
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

struct ACCEL
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};


typedef struct ICM_20948
{
    struct GYRO gyro;
    struct ACCEL accel;

} ICM_20948;

void ICM_gyro_config(uint8_t FS_config, uint8_t Avg_config, uint8_t DLPF_config);

void ICM_accel_config(uint8_t FS_config, uint8_t Avg_config, uint8_t DLPF_config);

int ICM_init(uint8_t Comm_protocol);

#endif ICM_20948_H