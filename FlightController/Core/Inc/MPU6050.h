/*
 * MPU6050.h
 *
 *  Created on: Mar 11, 2024
 *      Author: MAN-MADE
 *
 *      Здесь объявляются константы и прототипы функций для работы с гироскопом-акселерометром MPU6050
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define MPU6050_ADDR 0xD0 //адресс устройства 0х68 сдвинутый на 1 бит влево
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75 //айди который возвращает устройство, должно быть такое же как ADDR

#define GYRO_FS_SEL_250 0x00
#define GYRO_FS_SEL_500 0x01
#define GYRO_FS_SEL_1000 0x10
#define GYRO_FS_SEL_2000 0x11

#define ACCEL_FS_SEL_2g 0x00
#define ACCEL_FS_SEL_4g 0x01
#define ACCEL_FS_SEL_8g 0x10
#define ACCEL_FS_SEL_16g 0x11

struct accel
{
	 int16_t x;
	 int16_t y;
	 int16_t z;
};

struct gyro
{
	 int16_t x;
	 int16_t y;
	 int16_t z;
};

struct gyro_calib
{
	 float x;
	 float y;
	 float z;
};

int8_t MPU6050_Init (I2C_HandleTypeDef hi2c1, UART_HandleTypeDef huart2 ,uint8_t gyro_mode, uint8_t accel_mode);
struct accel MPU6050_Read_Accel (I2C_HandleTypeDef hi2c1,UART_HandleTypeDef huart2);
struct gyro MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c1,UART_HandleTypeDef huart2);
struct gyro_calib MPU6050_Calib_Gyro(I2C_HandleTypeDef hi2c1,UART_HandleTypeDef huart2);


#endif /* INC_MPU6050_H_ */
