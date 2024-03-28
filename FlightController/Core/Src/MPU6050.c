/*
 * MPU6050.c
 *
 *  Created on: Mar 11, 2024
 *      Author: MAN-MADE
 *
 *      Здесь определяется функционал работы с гироскопом-акселерометром MPU6050
 */

#include "main.h"


int8_t MPU6050_Init (I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2,uint8_t gyro_mode, uint8_t accel_mode)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);
	char str[10];
    sprintf(str,"%d \n\r", check);
    HAL_UART_Transmit(&huart2, (uint8_t*)str, sizeof(str),10);

	if (check == 114)  // почему то возвращает адрес 114, хотя должен 104
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &accel_mode, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &gyro_mode, 1, 1000);

		return 1;
	}
	else
	{
		char info[] = "Init Error\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 1);

		return 0;
	}

}

struct accel MPU6050_Read_Accel (I2C_HandleTypeDef hi2c1,UART_HandleTypeDef huart2)
{
	uint8_t Rec_Data[6];
	struct accel data;
	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	data.x = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	data.y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	data.z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	return data;
}

struct gyro MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c1,UART_HandleTypeDef huart2)
{
	uint8_t Rec_Data[6];
	struct gyro data;
	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	data.x = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	data.y = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	data.z = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	return data;
}

struct gyro_calib MPU6050_Calib_Gyro(I2C_HandleTypeDef hi2c1,UART_HandleTypeDef huart2)
{
	int8_t x_ok = 0;
	int8_t y_ok = 0;
	int8_t z_ok = 0;

	int16_t x_calib = 0;
	int16_t y_calib = 0;
	int16_t z_calib = 0;

	struct gyro_calib Cdata;
	struct gyro Gdata;

	while(!x_ok && !y_ok && !z_ok)
	{
		Gdata = MPU6050_Read_Gyro(hi2c1,huart2);

		x_calib+=(Gdata.x-x_calib) * 0.0000005;
	    y_calib+=(Gdata.y-y_calib) * 0.0000005;
	    z_calib+=(Gdata.z-z_calib) * 0.0000005;

	    if(abs(Gdata.x-x_calib) == 0) x_ok=1;
	    if(abs(Gdata.y-y_calib) == 0) y_ok=1;
	    if(abs(Gdata.z-z_calib) == 0) z_ok=1;

	    char cstr[32];
	    sprintf(cstr, "%d, %d, %d ", z_calib,Gdata.z,Gdata.z-z_calib);
	    HAL_UART_Transmit(&huart2, (uint8_t*) cstr, sizeof(cstr), 1);
	    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", sizeof("\r\n"), 1);
	}
	Cdata.x = x_calib;
	Cdata.y = y_calib;
	Cdata.x = x_calib;
	return Cdata;
}
