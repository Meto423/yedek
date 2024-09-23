/*
 * icm42688p.c
 *
 *  Created on: Sep 20, 2024
 *      Author: metin-demirci
 */

#include "icm42688p.h"
#include "main.h"


static SPI_HandleTypeDef* hspi_icm42688;

void ICM42688_Init(SPI_HandleTypeDef* hspi){

	hspi_icm42688 = hspi;

	// Write to power management to enable the sensor
	ICM42688_WriteRegister(ICM42688_PWR_MGMT0,0x0F);

	// Configure Accelerometer and Gyroscope ranges
	ICM42688_WriteRegister(0x4F,0x03); // Gyro full-scale range +-1000 dps
	ICM42688_WriteRegister(0x50,0x003); //Accelerometer range 	+-8g

}

void ICM42688_WriteRegister(uint8_t reg,uint8_t value){

	uint8_t data[2] = {reg & 0x7F,value}; // Clear MSB for write

	HAL_GPIO_WritePin(IMU1_CS_GPIO_Port,IMU1_CS_Pin,GPIO_PIN_RESET);// CS low
	HAL_SPI_Transmit(hspi_icm42688, (uint8_t*) data, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET);// CS high

}

uint8_t ICM42688_ReadRegister(uint8_t reg){

	uint8_t value = 0;
	uint8_t regAddr = reg | 0x80; // Set MSB for read

	HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_RESET); // CS low
	HAL_SPI_TransmitReceive(hspi_icm42688, &regAddr, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU1_CS_GPIO_Port, IMU1_CS_Pin, GPIO_PIN_SET); // CS high

	return value;

}

void ICM42688_ReadAccelData(int16_t* accelData){

	accelData[0] = (ICM42688_ReadRegister(ICM42688_ACCEL_DATA_X1) << 8) |
					ICM42688_ReadRegister(ICM42688_ACCEL_DATA_X0);

	accelData[1] = (ICM42688_ReadRegister(ICM42688_ACCEL_DATA_Y1) << 8) |
						ICM42688_ReadRegister(ICM42688_ACCEL_DATA_Y0);

	accelData[2] = (ICM42688_ReadRegister(ICM42688_ACCEL_DATA_Z1) << 8) |
						ICM42688_ReadRegister(ICM42688_ACCEL_DATA_Z0);
}

void ICM42688_ReadGyroData(int16_t* gyroData){

	gyroData[0] = (ICM42688_ReadRegister(ICM42688_GYRO_DATA_X1)<<8) |
					ICM42688_ReadRegister(ICM42688_GYRO_DATA_X0);

	gyroData[1] = (ICM42688_ReadRegister(ICM42688_GYRO_DATA_Y1)<<8) |
						ICM42688_ReadRegister(ICM42688_GYRO_DATA_Y0);

	gyroData[2] = (ICM42688_ReadRegister(ICM42688_GYRO_DATA_Z1)<<8) |
						ICM42688_ReadRegister(ICM42688_GYRO_DATA_Z0);

}

void ICM42688_ReadTempData(int16_t* tempData){

	*tempData = (ICM42688_ReadRegister(ICM42688_TEMP_DATA1) << 8) |
				ICM42688_ReadRegister(ICM42688_TEMP_DATA0);

}























