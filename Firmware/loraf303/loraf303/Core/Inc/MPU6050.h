#include "stm32f3xx_hal.h"


#include "HMC5883L.h"

//MPU6050 structure
typedef struct {
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	double Ax;
	double Ay;
	double Az;
	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	double Gx;
	double Gy;
	double Gz;
	float Temperature;
//	double KalmanAngleX;
//	double KalmanAngleY;
//	double KalmanAngleZ;
	float PositionX;
	float PositionY;
	float PositionZ;
	//Angle Rotatin Around X-Y-Z
	float Angle[3];

} MPU6050;

typedef struct{
	float ErrorGyroX;
	float ErrorGyroY;
	float ErrorGyroZ;
	float ErrorAccX;
	float ErrorAccY;
	float ErrorAccZ;
}MPU6050Error;

//Kalman Filter structure
typedef struct {
	double Q_angle;
	double Q_bias;
	double R_measure;
	double angle;
	double bias;
	double P[2][2];

}KalmanFilter;

uint8_t MPU6050_Init(void);

void MPU6050_Read_Accel(MPU6050 *DataStruct);

void MPU6050_Read_Gyro(MPU6050 *DataStruct);

void MPU6050_Read_Temp(MPU6050 *DataStruct);

void MPU6050_Read_All(MPU6050 *DataStruct, MPU6050Error *DataError, HMC5883L *DataStruct2);

float MPU6050_GetDistance(MPU6050 *DataStruct, double dt);

void MPU6050_Calibration(MPU6050 *DataStruct,MPU6050Error *DataError);

double Kalman_Angle(KalmanFilter *Kalman, double newAngle, double newRate, double dt);
