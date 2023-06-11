#include "stm32f1xx_hal.h"
// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

void MPU6050_init(void); //Initialize the MPU
void MPU6050_Read_Accel (float *Ax, float *Ay, float *Az); //Read MPU Accelerator
void MPU6050_Read_Gyro (float *Gx, float *Gy, float *Gz); //Read MPU Gyroscope
double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);


