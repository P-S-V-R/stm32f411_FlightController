/*
 * mpu6050.h
 *
 *  Created on: Feb 21, 2024
 *      Author: Srinivas
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "stdbool.h"
#include "math.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

I2C_HandleTypeDef hi2c1;

#define MPU6050_ADDR 0xD0
#define DLPF_CFG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;


bool set_gyro_angle = true;

long gyro_x_cal;
long gyro_y_cal;
long gyro_z_cal;

float angle_pitch_acc_cal;
float angle_roll_acc_cal;

long gyro_x_man = 278;
long gyro_y_man = -2898;
long gyro_z_man = 352;

long gyro_x;
long gyro_y;
long gyro_z;


long acc_x;
long acc_y;
long acc_z;
long acc_total_vector;

float angle_roll;
float angle_pitch;

//float angle_roll_output;
//float angle_pitch_output;

float angle_roll_acc;
float angle_pitch_acc;

float roll_level_adjust;
float pitch_level_adjust;

float gyro_roll_input;
float gyro_pitch_input;
float gyro_yaw_input;

float gain_pitch;
float gain_roll;

float state_pitch;
float state_roll;

float uncertainity_pitch;
float uncertainity_roll;

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x03;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, DLPF_CFG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 8g
		Data = 0x10;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 500 �/s
		Data = 0x08;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void gyro_get_data(){

	uint8_t Accel_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Accel_Val_Raw, 6, 1000);

	acc_y = (int16_t) (Accel_Val_Raw[0] << 8 | Accel_Val_Raw [1]);
	acc_x = (int16_t) (Accel_Val_Raw[2] << 8 | Accel_Val_Raw [3]);
	acc_z = (int16_t) (Accel_Val_Raw[4] << 8 | Accel_Val_Raw [5]);

	uint8_t Gyro_Val_Raw[6];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Gyro_Val_Raw, 6, 1000);

	gyro_y = (int16_t) (Gyro_Val_Raw[0] << 8 | Gyro_Val_Raw [1]);
	gyro_x = (int16_t) (Gyro_Val_Raw[2] << 8 | Gyro_Val_Raw [3]);
	gyro_z = (int16_t) (Gyro_Val_Raw[4] << 8 | Gyro_Val_Raw [5]);

	gyro_x *= -1;
	//gyro_y *= -1;
	gyro_z *= -1;

}

void mpu6050_cal(){

	for( int i = 0; i < 2000; i++){
		if ( i % 15 == 0 ) HAL_GPIO_TogglePin(GPIOC, led_cal_Pin);
		gyro_get_data();

		gyro_x_cal += gyro_x;
		gyro_y_cal += gyro_y;
		gyro_z_cal += gyro_z;


		acc_total_vector = sqrt( ( acc_x*acc_x ) + ( acc_y * acc_y) + ( acc_z * acc_z ) );

		  if ( abs(acc_y) < acc_total_vector ){
			  angle_pitch_acc = asin( (float) acc_y / acc_total_vector ) * 57.296;
		  }

		  if ( abs(acc_x) < acc_total_vector ){
			  angle_roll_acc = asin( (float) acc_x / acc_total_vector ) * 57.296;
		  }

		  angle_pitch_acc_cal += angle_pitch_acc;
		  angle_roll_acc_cal += angle_roll_acc;

		HAL_Delay(4);

	}

	gyro_x_cal /= 2000;
	gyro_y_cal /= 2000;
	gyro_z_cal /= 2000;

	angle_pitch_acc_cal /= 2000;
	angle_roll_acc_cal /= 2000;
}

void get_angles(){
	gyro_get_data();

		  gyro_x -= gyro_x_cal;
		  gyro_y -= gyro_y_cal;
		  gyro_z -= gyro_z_cal;

	//	  acc_x -= acc_x_cal;
	//	  acc_y -= acc_y_cal;
	//	  acc_z -= acc_z_cal;

		  gyro_pitch_input 	= ( gyro_pitch_input * 0.7 ) + ((float)( gyro_x / 65.5) * 0.3);
		  gyro_roll_input 	= ( gyro_roll_input * 0.7 ) + ((float)( gyro_y / 65.5) * 0.3);
		  gyro_yaw_input 	= ( gyro_yaw_input * 0.7 ) + ((float)( gyro_z / 65.5) * 0.3);

		  angle_pitch += gyro_x * 0.0000611;
		  angle_roll += gyro_y * 0.0000611;


		  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);
		  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);

		  acc_total_vector = sqrt( ( acc_x*acc_x ) + ( acc_y * acc_y) + ( acc_z * acc_z ) );

		  if ( abs(acc_y) < acc_total_vector ){
			  angle_pitch_acc = asin( (float) acc_y / acc_total_vector ) * 57.296;
			  angle_pitch_acc -= angle_pitch_acc_cal;
		  }

		  if ( abs(acc_x) < acc_total_vector ){
			  angle_roll_acc = asin( (float) acc_x / acc_total_vector ) * 57.296;
			  angle_roll_acc -= angle_roll_acc_cal;
		  }

		  state_pitch = state_pitch + 0.004*gyro_pitch_input;
		  uncertainity_pitch = uncertainity_pitch + 0.004*0.004*4*4;
		  gain_pitch = uncertainity_pitch /(uncertainity_pitch + 3*3);
		  state_pitch = state_pitch + gain_pitch*(angle_pitch_acc - state_pitch);
		  uncertainity_pitch = (1 - gain_pitch) * uncertainity_pitch;
		  angle_pitch = state_pitch;

		  state_roll = state_roll + 0.004*gyro_roll_input;
		  uncertainity_roll = uncertainity_roll + 0.004*0.004*4*4;
		  gain_roll = uncertainity_roll /(uncertainity_roll + 3*3);
		  state_roll = state_roll + gain_roll*(angle_roll_acc - state_roll);
		  uncertainity_roll = (1 - gain_roll) * uncertainity_roll;
		  angle_roll = state_roll;

//		  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
//		  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

		  pitch_level_adjust = angle_pitch * 15;
		  roll_level_adjust = angle_roll * 15;
}


#endif /* INC_MPU6050_H_ */
