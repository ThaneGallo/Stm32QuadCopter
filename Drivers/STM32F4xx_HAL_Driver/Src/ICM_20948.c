#include "stm32f4xx_hal_gpio.c"
#include "stm32f4xx_hal_spi.c"
#include "stm32f4xx_hal_i2c.c"

#include "ICM_20948.h"


void ICM_gyro_config(uint8_t FS_config, uint8_t Avg_config, uint8_t DLPF_config)
{
    // set reg bank 2

    ICM_20948 sensor;

    uint8_t GYRO_CONFIG_1 = FS_config | DLPF_config;
    uint8_t GYRO_CONFIG_2 = Avg_config;
}

void ICM_accel_config(uint8_t FS_config, uint8_t Avg_config, uint8_t DLPF_config)
{
    // set reg bank 2

    uint8_t ACCEL_CONFIG_1 = FS_config | DLPF_config;
    uint8_t ACCEL_CONFIG_2 = Avg_config;
}

int ICM_init(uint8_t Comm_protocol)
{

    // set reg bank 0
    // assert 0x00 is 0xEA

    // misc config

    if (Comm_protocol == ICM_I2C)
    {
    }

    else if (Comm_protocol == ICM_SPI)
    {
    }

    else
    {
        return -1; // invalid Comm_protocol
    }

    // interrupts

    // wakeup from sleep mode last thing
    // clear bit 6 in addr 0x06

    return 0;
}

ICM_read_gyro(ICM_20948* sensor){
//set bank 0




}

ICM_read_accel(ICM_20948* sensor){
//set bank 0


}