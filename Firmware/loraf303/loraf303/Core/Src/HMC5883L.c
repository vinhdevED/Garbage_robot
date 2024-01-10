/*
 * HMC5883L.c
 *
 *  Created on: Jun 30, 2023
 *      Author: trand
 */
#include "HMC5883L.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;


// Function to write a byte to a register on the HMC5883L module
void HMC5883L_WriteRegister(uint8_t regAddr, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_DEFAULT_ADDRESS, regAddr, 1, &data, 1, HAL_MAX_DELAY);
}

////Function to read multiple bytes from consecutive registers on the HMC5883L module
//void HMC5883L_ReadRegisters(uint8_t regAddr, uint8_t *buffer, uint16_t count) {
//    HAL_I2C_Mem_Read(&hi2c2, HMC5883L_DEFAULT_ADDRESS, regAddr, 1, buffer, count, HAL_MAX_DELAY);
//}
int16_t HMC5883L_ReadRegisters(uint8_t regAddr) {
	int16_t value;
    uint8_t vha[2];
    HAL_I2C_Mem_Read(&hi2c1, HMC5883L_DEFAULT_ADDRESS, regAddr, 1, vha, 2, HAL_MAX_DELAY);
    value = vha[0] << 8 | vha[1];

    return value;
}

// Function to initialize the HMC5883L module
void HMC5883L_Init(void) {
    // Configure the module's settings
    // For example, set the measurement mode, output rate, etc.
    // Write the appropriate values to the corresponding registers

    // Set the Configuration Register A (0x00)
    uint8_t configA = 0x10; // Configurations: 75 Hz output rate,
    HMC5883L_WriteRegister(HMC5883L_REG_CONFIG_A, configA);

    // Set the Configuration Register B (0x01)
    uint8_t configB = 0x20; // Gain = 1.3 Ga
    HMC5883L_WriteRegister(HMC5883L_REG_CONFIG_B, configB);

    // Set the Mode Register (0x02)
    uint8_t mode = 0x00; // Continuous measurement mode
    HMC5883L_WriteRegister(HMC5883L_REG_MODE, mode);
}

// Function to read the magnetic field data from the HMC5883L module
void HMC5883L_ReadMagneticField(HMC5883L *hmc5883l){
//	uint8_t buffer[6];
//	HMC5883L_ReadRegisters(HMC5883L_REG_OUT_X_M, buffer, 6);
//	// Convert the raw data to signed 16-bit integers
//	hmc5883l->XAxis = (int16_t)((buffer[0] << 8) | buffer[1]);
//	hmc5883l->YAxis = (int16_t)((buffer[2] << 8) | buffer[3]);
//	hmc5883l->ZAxis = (int16_t)((buffer[4] << 8) | buffer[5]);
	hmc5883l->XAxis = HMC5883L_ReadRegisters(HMC5883L_REG_OUT_X_M);
	hmc5883l->YAxis = HMC5883L_ReadRegisters(HMC5883L_REG_OUT_Y_M);
	hmc5883l->ZAxis = HMC5883L_ReadRegisters(HMC5883L_REG_OUT_Z_M);
}


void HMC5883L_Calibration(HMC5883L *hmc5883l){
	int32_t x_offset = 0, y_offset = 0, z_offset = 0;
	int32_t x_max = INT32_MIN, y_max = INT32_MIN, z_max = INT32_MIN;
	int32_t x_min = INT32_MAX, y_min = INT32_MAX, z_min = INT32_MAX;
	for(int i=0;i<1000;i++){
		    HMC5883L_ReadMagneticField(hmc5883l);
	        //Update data max and min
			if(hmc5883l->XAxis < x_min) x_min = hmc5883l->XAxis;
			if(hmc5883l->XAxis > x_max) x_max = hmc5883l->XAxis;
			if(hmc5883l->YAxis < y_min) y_min = hmc5883l->YAxis;
			if(hmc5883l->YAxis > y_max) y_max = hmc5883l->YAxis;
			if(hmc5883l->ZAxis < z_min) x_min = hmc5883l->ZAxis;
			if(hmc5883l->ZAxis > z_max) z_max = hmc5883l->ZAxis;

			HAL_Delay(10);
	}
	//Calculate offset from min and max
	x_offset = (x_max + x_min) / 2;
	y_offset = (y_max + y_min) / 2;
	z_offset = (z_max + z_min) / 2;

	//Finally
	 hmc5883l->XAxis  -= x_offset;
	 hmc5883l->YAxis -= y_offset;
     hmc5883l->ZAxis -= z_offset;

}

double HMC5883L_GetHeading(HMC5883L *hmc5883l,char option){
	HMC5883L_ReadMagneticField(hmc5883l);
	double heading = atan2(hmc5883l->YAxis,hmc5883l->XAxis);
		double headingDegree = heading *180 /M_PI;

	switch (option) {
		case 'A':  //ANGLE
			heading += DECLINATION_ANGLE_DEGREE;
			 if (headingDegree < 0) {
				 headingDegree += 360;
			 }

		break;
		case 'R': //RADIAN
			heading += DECLINATION_ANGLE_RAD;
			if(heading <0){
					heading += 2* M_PI;
			}
		    if (heading > 2 * M_PI){
				    heading -= 2 * M_PI;
		    }
		break;
	}
	return headingDegree;
}



