#include "MPU6050.h"
#include <math.h>
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define INT_ENABLE 0x38
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;



extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly
void MPU6050_init(void)
{
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);
	if (check == 104)
	{
		//Power management register write all 0's to wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
		//Set data rate of 1KHz by writing SMPRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
		//Writing both register with 0 to set full scale range
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

//		//Enable data ready interrupts
//		data = 0x01;
//		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, INT_ENABLE, 1, &data, 1, 1000);
	}

}

//Function with multiple return using pointer

void MPU6050_Read_Accel (float* Ax, float* Ay, float* Az)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	//Adding 2 BYTES into 16 bit integer
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	*Ax = Accel_X_RAW*100/16384.0;
	*Ay = Accel_Y_RAW*100/16384.0;
	*Az = Accel_Z_RAW*100/16384.0;
}
void MPU6050_Read_Gyro (float* Gx, float* Gy, float* Gz)
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [1]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [1]);
	*Gx = Gyro_X_RAW/131.0;
	*Gy = Gyro_Y_RAW/131.0;
	*Gz = Gyro_Z_RAW/131.0;
}

