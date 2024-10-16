/*
 * ICM20948.h
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

#ifndef ICM20948_H_
#define ICM20948_H_

#include <stdint.h>

#define SPI_BUS (&hspi1)   // ***
#define UART_BUS (&huart3) // ***


#define GYRO_SEL (1 << 0)
#define ACCEL_SEL (1 << 1)
#define MAGN_SEL (1 << 2)

#define USER_BANK_SEL (0x7F)
#define USER_BANK_0 (0x00)
#define USER_BANK_1 (0x10)
#define USER_BANK_2 (0x20)
#define USER_BANK_3 (0x30)

#define PWR_MGMT_1 (0x06)
#define PWR_MGMT_2 (0x07)

#define GYRO_CONFIG_1 (0x01)
#define GYRO_CONFIG_2 (0x02)

#define ACCEL_CONFIG_1 (0x14)
#define ACCEL_CONFIG_2 (0x15)

#define CLK_BEST_AVAIL (0x01)
#define GYRO_LPF_17HZ (0x29)

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
    uint16_t sensitivity;
};

struct ACCEL
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t sensitivity;
};

struct MAGN
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t sensitivity;
};

struct CONFG
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};

typedef struct ICM_20948
{
    struct CONFG config;
    struct GYRO gyro;
    struct ACCEL accel;
    struct MAGN magn;

} ICM_20948;

void ICM_PowerOn();
void ICM_SelectBank(uint8_t bank);
void ICM_ReadAccelGyro(ICM_20948 *IMU);
void ICM_ReadMag(ICM_20948 *IMU);

void ICM_GyroInit(ICM_20948 *IMU, uint8_t FS_config, uint8_t LPF_config, uint8_t AVG_config);
void ICM_AccelInit(ICM_20948 *IMU, uint8_t FS_config, uint8_t LPF_config, uint8_t AVG_config);
void ICM_MagnInit();

void ICM_SelectBank(uint8_t bank);
void ICM_Disable_I2C(void);
void ICM_CSHigh(void);
void ICM_CSLow(void);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroLPF(uint8_t lpf);

#endif ICM20948_H_
