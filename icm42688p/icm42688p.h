#ifndef _ICM42688P_H_
#define _ICM42688P_H_

#include "stm32h7xx_hal.h"

//Register addresses

#define ICM42688_WHO_AM_I 0x75
#define ICM42688_PWR_MGMT0 0x4E

// Accelerometer data registers
#define ICM42688_ACCEL_DATA_X1 0x1F
#define ICM42688_ACCEL_DATA_X0 0x20
#define ICM42688_ACCEL_DATA_Y1 0x21
#define ICM42688_ACCEL_DATA_Y0 0x22
#define ICM42688_ACCEL_DATA_Z1 0x23
#define ICM42688_ACCEL_DATA_Z0 0x24

// Gyroscope data registers
#define ICM42688_GYRO_DATA_X1 0x25
#define ICM42688_GYRO_DATA_X0 0x26
#define ICM42688_GYRO_DATA_Y1 0x27
#define ICM42688_GYRO_DATA_Y0 0x28
#define ICM42688_GYRO_DATA_Z1 0x29
#define ICM42688_GYRO_DATA_Z0 0x2A

// Temperature data registers
#define ICM42688_TEMP_DATA1 0x39
#define ICM42688_TEMP_DATA0 0x3A

// Function prototypes

void ICM42688_Init(SPI_HandleTypeDef* hspi);
void ICM42688_WriteRegister(uint8_t reg, uint8_t value);
uint8_t ICM42688_ReadRegister(uint8_t reg);
void ICM42688_ReadAccelData(int16_t* accelData);
void ICM42688_ReadGyroData(int16_t* gyroData);
void ICM42688_ReadTempData(int16_t* tempData);

#endif











