#include "MPU6050.h"
#include "HMC5883L.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;

#define RAD_TO_DEG 180/M_PI
#define DEG_TO_RAD M_PI/180

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define INT_ENABLE 0x38
//Set up MPU6050
#define MPU6050_ADDR 0xD0
// Complementary filter constants
#define ALPHA 0.96 // Complementary filter coefficient


const uint32_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t  timer;
uint32_t previousTime;
int c;
double accAngleX, accAngleY,accAngleZ,gyroAngleX, gyroAngleY ,gyroAngleZ;


KalmanFilter KalmanX ={
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};
KalmanFilter KalmanY ={
		.Q_angle = 0.001f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};





uint8_t MPU6050_Init(void){
	uint8_t check,data;
	//check device ID WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR , WHO_AM_I_REG,1, &check, 1 , i2c_timeout);
	if (check == 104)
		{
			//Power management register write all 0's to wake up sensor
			data = 0;
			HAL_I2C_Mem_Write(&hi2c1,MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, i2c_timeout);
			//Set data rate of 1KHz by writing SMPRT_DIV register
			data = 0x07;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, i2c_timeout);
			//Writing both register with 0 to set full scale range
			data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, i2c_timeout);

			data = 0x00;
			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, i2c_timeout);
			return 0;
		}
	return 1;
}

void MPU6050_Read_Accel (MPU6050 *mpu6050)
{
	uint8_t Rec_Data[6];

	 // Read 6 BYTES of data starting from ACCEL_XOUT_H register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
	//Adding 2 BYTES into 16 bit integer
	mpu6050->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	mpu6050->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	mpu6050->Ax = mpu6050->Accel_X_RAW/16384.0;
	mpu6050->Ay = mpu6050->Accel_Y_RAW/16384.0;
	mpu6050->Az = mpu6050->Accel_Z_RAW/Accel_Z_corrector;
}

void MPU6050_Read_Gyro (MPU6050 *mpu6050)
{
	uint8_t Rec_Data[6];
	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);
	mpu6050->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	mpu6050->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	mpu6050->Gx = mpu6050->Gyro_X_RAW/131.0;
	mpu6050->Gy = mpu6050->Gyro_Y_RAW/131.0;
	mpu6050->Gz = mpu6050->Gyro_Z_RAW/131.0;
}


void MPU6050_Read_All(MPU6050 *mpu6050, MPU6050Error *error , HMC5883L *hmc5883l){
	uint8_t Rec_Data[14];
		  int16_t temp;
		 // Read 14 BYTES of data starting from ACCEL_XOUT_H register
		  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);
		  mpu6050->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		  mpu6050->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
		  mpu6050->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
		  temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
		  mpu6050->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
		  mpu6050->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
		  mpu6050->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);
		  mpu6050->Ax = mpu6050->Accel_X_RAW / 16384.0;
		  mpu6050->Ay = mpu6050->Accel_Y_RAW / 16384.0;
		  mpu6050->Az = mpu6050->Accel_Z_RAW / 16384.0;
		  mpu6050->Temperature =(float)(temp/(float)340.0+(float)36.53);
		  mpu6050->Gx = mpu6050->Gyro_X_RAW / 131.0;
		  mpu6050->Gy = mpu6050->Gyro_Y_RAW / 131.0;
		  mpu6050->Gz = mpu6050->Gyro_Z_RAW / 131.0;




	  double dt = (double)(HAL_GetTick() - previousTime)/1000;
	  previousTime = HAL_GetTick();

	  accAngleX = (atan2(-mpu6050->Ay,mpu6050->Az)*RAD_TO_DEG) - error->ErrorAccX;
	  accAngleY = (atan2(-mpu6050->Ax,sqrt(pow(mpu6050->Az,2)+pow(mpu6050->Ay,2)))*RAD_TO_DEG) - error->ErrorAccY;


	  mpu6050->Gx -= error->ErrorGyroX;
	  mpu6050->Gy -= error->ErrorGyroY;
	  mpu6050->Gz -= error->ErrorGyroZ;

	  mpu6050->Angle[0] = Kalman_Angle(&KalmanX, accAngleX, mpu6050->Gx, dt);
	  mpu6050->Angle[1] = Kalman_Angle(&KalmanY, accAngleY, mpu6050->Gy, dt);
	  double magx = hmc5883l->XAxis*sin( mpu6050->Angle[0]*DEG_TO_RAD)-hmc5883l->YAxis*cos( mpu6050->Angle[0]*DEG_TO_RAD);
	  double magy = hmc5883l->XAxis*cos(mpu6050->Angle[1]*DEG_TO_RAD)+ hmc5883l->YAxis*sin(mpu6050->Angle[1]*DEG_TO_RAD)*sin(mpu6050->Angle[0]*DEG_TO_RAD)+hmc5883l->ZAxis*sin(mpu6050->Angle[1]*DEG_TO_RAD)*cos(mpu6050->Angle[0]*DEG_TO_RAD);

	  double heading = atan2(magx,magy)*RAD_TO_DEG;
	  // Adjust heading to be in the range [0, 360]
	  if(heading < 0) {
	      heading += 360;
	  }
	  heading = 360 - heading;
	  mpu6050->Angle[2] = heading;

}

double Kalman_Angle(KalmanFilter *kalman ,double newAngle, double newRate, double dt){
	double rate = newRate - kalman->bias;
	kalman->angle += dt*rate;

	kalman->P[0][0] += dt*(dt*kalman->P[1][1] -kalman->P[0][1]-kalman->P[1][0]+kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += kalman->Q_bias * dt;
	//Kalman Gain Kn(Độ lợi)
	double S1 = kalman->P[0][0] + kalman->R_measure;
	double S2 = kalman->P[1][0] + kalman->R_measure;
	double K[2];
	K[0] = kalman->P[0][0] / S1;
	K[1] = kalman->P[1][0] / S2;
    double y = newAngle - kalman->angle;
    //Multiplying K[0] for the difference y
    //we take the value that controlling to update estimate angle
	kalman->angle += K[0] * y;
	kalman->bias += K[1] * y;

	double P00_temp = kalman->P[0][0];
	double P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;


	return kalman->angle;
}

float MPU6050_GetDistance(MPU6050 *mpu6050,double dt){
	float velocityX = 0.0;
	float posX = 0.0;
	velocityX += mpu6050->Ax*dt;
	posX += velocityX*dt;
	return posX;
}

void MPU6050_Calibration(MPU6050 *mpu6050,MPU6050Error *error){

	    c=0;
	    while(c<200){
	    	      MPU6050_Read_Accel(mpu6050);
	    	      HAL_Delay(1);
	    	      error->ErrorAccX += (atan2(-mpu6050->Ay,mpu6050->Az)*RAD_TO_DEG);
	    	      error->ErrorAccY += (atan2(-mpu6050->Ax,sqrt(pow(mpu6050->Az,2)+pow(mpu6050->Ay,2)))*RAD_TO_DEG);
	    	      c++;
	    }
	    error->ErrorAccX = error->ErrorAccX /200;
	    error->ErrorAccY = error->ErrorAccY /200;

	    c =0;
	    while(c<200){
	      MPU6050_Read_Gyro(mpu6050);
	      HAL_Delay(1);
	      error->ErrorGyroX += mpu6050->Gx;
	      error->ErrorGyroY += mpu6050->Gy;
	      error->ErrorGyroZ += mpu6050->Gz;
	      c++;
	    }
	    error->ErrorGyroX = error->ErrorGyroX /200;
	    error->ErrorGyroY = error->ErrorGyroY /200;
	    error->ErrorGyroZ = error->ErrorGyroZ /200;
}

