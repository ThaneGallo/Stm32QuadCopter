/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2024
 *      Author: Thane
 */

// *** Three asterisks to the side of a line means this may change based on platform
#include "main.h"			   // ***
#include "stm32f4xx_hal_spi.h" // ***
#include "stm32f4xx_hal_gpio.h"
#include "ICM20948.h"

/*
@brief Reads data from a desired address
@param reg base register address
@param pData data pointer that is to be read
@param Size size of data to be read
*/
void ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg | 0x80;
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
	HAL_SPI_Receive_DMA(SPI_BUS, pData, Size);
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
}

/*
@brief Writes data to a desired address
@param reg base register address
@param pData data pointer that is to be written
@param Size size of data to be written
*/
void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg & 0x7F;
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
	HAL_SPI_Transmit_DMA(SPI_BUS, pData, Size);
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
}

/*
@brief Reads a single byte at desired address
@param reg register address
@return value read at register
*/
uint8_t ICM_ReadOneByte(uint8_t reg) // ***
{
	reg = reg | 0x80;
	uint8_t pData;

	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
	while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Receive_DMA(SPI_BUS, &pData, 1);
	while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);

	return pData;
}

/*
@brief Writes a single byte at desired address
@param reg register address
*/
void ICM_WriteOneByte(uint8_t reg, uint8_t Data) // ***
{
	reg = reg & 0x7F;
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
	HAL_SPI_Transmit_DMA(SPI_BUS, &Data, 1);
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
}

/*
@brief Writes value using i2c to magnometer register
@param reg register to write to 
@param value value to be written 
 */
void i2c_Mag_write(uint8_t reg, uint8_t value)
{
	ICM_WriteOneByte(0x7F, 0x30);
	HAL_Delay(1);
	ICM_WriteOneByte(0x03, 0x0C); // mode: write
	HAL_Delay(1);
	ICM_WriteOneByte(0x04, reg); // set reg addr
	HAL_Delay(1);
	ICM_WriteOneByte(0x06, value); // send value

	HAL_Delay(1);
}

/*
@brief Reads value using i2c to magnometer register
@param reg register to read 
@return value read at register
 */
static uint8_t ICM_Mag_Read(uint8_t reg)
{
	uint8_t Data;
	ICM_WriteOneByte(0x7F, 0x30);
	HAL_Delay(1);
	ICM_WriteOneByte(0x03, 0x0C | 0x80);
	HAL_Delay(1);
	ICM_WriteOneByte(0x04, reg); // set reg addr
	HAL_Delay(1);
	ICM_WriteOneByte(0x06, 0xff); // read
	HAL_Delay(1);
	ICM_WriteOneByte(0x7F, 0x00);
	ICM_ReadOneByte(0x3B, &Data);
	HAL_Delay(1);
	return Data;
}

/*
@brief Reads and stores x, y, and z magneometer values into IMU struct
@param IMU pointer to IMU chip struct
*/
void ICM_ReadMag(struct ICM_20948 *IMU)
{
	uint8_t mag_buffer[9];

	assert(0x09 == ICM_Mag_Read(0x01)); // verifies device id

	mag_buffer[0] = ICM_Mag_Read(0x11);
	mag_buffer[1] = ICM_Mag_Read(0x12);
	IMU->magn.x = mag_buffer[0] | mag_buffer[1] << 8;

	mag_buffer[2] = ICM_Mag_Read(0x13);
	mag_buffer[3] = ICM_Mag_Read(0x14);
	IMU->magn.y = mag_buffer[2] | mag_buffer[3] << 8;

	mag_buffer[4] = ICM_Mag_Read(0x15);
	mag_buffer[5] = ICM_Mag_Read(0x16);
	IMU->magn.z = mag_buffer[4] | mag_buffer[5] << 8;

	i2c_Mag_write(0x31, 0x01);
}

/*
@brief Turns on sensors within IMU
@param sensorChoice sensors to be turned on
*/
void ICM_SensorOn(uint8_t sensorChoice)
{

	if (ACCEL_SEL & sensorChoice)
	{
		// bits 5:3
		ICM_WriteOneByte(PWR_MGMT_2, 0x38 | ICM_ReadOneByte(PWR_MGMT_2));
	}

	if (GYRO_SEL & sensorChoice)
	{
		// bits 2:0
		ICM_WriteOneByte(PWR_MGMT_2, 0x07 | ICM_ReadOneByte(PWR_MGMT_2));
	}

	if (MAGN_SEL & sensorChoice)
	{
		ICM_MagnInit();
	}
}

/*
@brief Turns off sensors within IMU
@param sensorChoice sensors to be turned off
*/
void ICM_SensorOff(uint8_t sensorChoice)
{

	if (ACCEL_SEL & sensorChoice)
	{
		// bits 5:3
		ICM_WriteOneByte(PWR_MGMT_2, (~0x38) | ICM_ReadOneByte(PWR_MGMT_2));
	}

	if (GYRO_SEL & sensorChoice)
	{
		// bits 2:0
		ICM_WriteOneByte(PWR_MGMT_2, (~0x07) | ICM_ReadOneByte(PWR_MGMT_2));
	}

	if (MAGN_SEL & sensorChoice)
	{
		ICM_MagnInit();
	}
}

/*
@brief First initialization function to begin chip config
@param sensorChoice sensors to be turned on
*/
void ICM_PowerOn(uint8_t sensorChoice)
{
	char uart_buffer[200];
	uint8_t *whoami;

	assert(0xEA == ICM_ReadOneByte(0x00, whoami));
	ICM_CSHigh();
	HAL_Delay(10);
	ICM_SelectBank(USER_BANK_0);
	HAL_Delay(10);
	ICM_Disable_I2C();
	HAL_Delay(10);
	ICM_SetClock((uint8_t)CLK_BEST_AVAIL);
	HAL_Delay(10);
	ICM_SensorOff(sensorChoice);
	HAL_Delay(20);
	ICM_SensorOn(sensorChoice);
	HAL_Delay(10);
}

/*
@brief initializes the magnetometer
*/
void ICM_MagnInit()
{

	ICM_SelectBank(USER_BANK_2);
	HAL_Delay(20);

	// Configure AUX_I2C Magnetometer (onboard ICM-20948)
	ICM_WriteOneByte(0x7F, 0x00); // Select user bank 0
	ICM_WriteOneByte(0x0F, 0x30); // INT Pin / Bypass Enable Configuration
	ICM_WriteOneByte(0x03, 0x20); // I2C_MST_EN
	ICM_WriteOneByte(0x7F, 0x30); // Select user bank 3
	ICM_WriteOneByte(0x01, 0x4D); // I2C Master mode and Speed 400 kHz
	ICM_WriteOneByte(0x02, 0x01); // I2C_SLV0 _DLY_ enable
	ICM_WriteOneByte(0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byte

	// Initialize magnetometer
	i2c_Mag_write(0x32, 0x01); // Reset AK8963
	HAL_Delay(1000);
	i2c_Mag_write(0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output
}

/*
@brief Initializes the gyroscope, to be called after power_on()
@param IMU pointer to IMU struct
@param FS_config Full scale size +- xg
@param LPF_config Lowpass filter config setting
@param AVG_config Number of samples to be taken per outputted value
*/
void ICM_GyroInit(ICM_20948 *IMU, uint8_t FS_config, uint8_t LPF_config, uint8_t AVG_config)
{
	switch (FS_config)
	{
	case GYRO_FS_250:
		IMU->gyro.sensitivity = 250;
		break;
	case GYRO_FS_500:
		IMU->gyro.sensitivity = 500;
		break;
	case GYRO_FS_1000:
		IMU->gyro.sensitivity = 1000;
		break;
	case GYRO_FS_2000:
		IMU->gyro.sensitivity = 2500;
		break;
	}

	// sets up gyro
	ICM_SelectBank(USER_BANK_2);
	HAL_Delay(20);
	ICM_WriteOneByte(GYRO_CONFIG_1, FS_config | LPF_config);
	HAL_Delay(10);
	ICM_WriteOneByte(GYRO_CONFIG_2, AVG_config);
	HAL_Delay(10);

	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	ICM_WriteOneByte(0x00, 0x0A);
	HAL_Delay(10);
}

/*
@brief Initializes the accelerometer, to be called after power_on()
@param IMU pointer to IMU struct
@param FS_config Full scale size +- xms
@param LPF_config Lowpass filter config setting
@param AVG_config Number of samples to be taken per outputted value
*/
void ICM_AccelInit(ICM_20948 *IMU, uint8_t FS_config, uint8_t LPF_config, uint8_t AVG_config)
{

	switch (FS_config)
	{
	case ACCEL_FS_2G:
		IMU->accel.sensitivity = 2;
		break;
	case ACCEL_FS_4G:
		IMU->accel.sensitivity = 4;
		break;
	case ACCEL_FS_8G:
		IMU->accel.sensitivity = 8;
		break;
	case ACCEL_FS_16G:
		IMU->accel.sensitivity = 16;
		break;
	}

	ICM_SelectBank(USER_BANK_2);
	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	ICM_WriteOneByte(ACCEL_CONFIG_1, (FS_config | LPF_config));
	HAL_Delay(10);

	ICM_WriteOneByte(ACCEL_CONFIG_2, AVG_config);
	HAL_Delay(10);

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	ICM_WriteOneByte(0x10, 0x00);
	HAL_Delay(10);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	ICM_WriteOneByte(0x11, 0x0A);
	HAL_Delay(10);
}

/*
@brief Reads and stores x, y, and z accelerometer values into IMU struct
@param IMU pointer to IMU struct
*/
void ICM_ReadAccel(ICM_20948 *IMU)
{
	uint8_t raw_data[6];
	ICM_readBytes(0x2D, raw_data, 6);

	IMU->accel.x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / IMU->accel.sensitivity;
	IMU->accel.y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / IMU->accel.sensitivity;
	IMU->accel.z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / IMU->accel.sensitivity;
}

/*
@brief Reads and stores x, y, and z gyroscope values into IMU struct
@param IMU pointer to IMU struct
*/
void ICM_ReadGyro(ICM_20948 *IMU)
{
	uint8_t raw_data[6];
	ICM_readBytes(0x33, raw_data, 6);

	IMU->gyro.x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / IMU->gyro.sensitivity;
	IMU->gyro.y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / IMU->gyro.sensitivity;
	IMU->gyro.z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / IMU->gyro.sensitivity;
}

/*
@brief Selects register bank
@param bank bank to select
*/
void ICM_SelectBank(uint8_t bank)
{
	ICM_WriteOneByte(USER_BANK_SEL, bank);
}

/*
@brief Disables I2C
*/
void ICM_Disable_I2C(void)
{
	ICM_WriteOneByte(0x03, 0x78);
}

/*
@brief Sets chip select to high
*/
void ICM_CSHigh(void)
{
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, SET);
}

/*
@brief Sets chip select to low
*/
void ICM_CSLow(void)
{
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, RESET);
}

/*
@brief Sets desired clock value
*/
void ICM_SetClock(uint8_t clk)
{
	ICM_WriteOneByte(PWR_MGMT_1, clk);
}
