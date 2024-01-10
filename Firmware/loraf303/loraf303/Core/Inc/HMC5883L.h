/*
 * HMC5883L.h
 *
 *  Created on: Jun 30, 2023
 *      Author: trand
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

#include "stm32f3xx_hal.h"

#define HMC5883L_ADDRESS              0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    (HMC5883L_ADDRESS<<1) // 0x3C

#define HMC5883L_REG_CONFIG_A         (0x00)
#define HMC5883L_REG_CONFIG_B         (0x01)
#define HMC5883L_REG_MODE             (0x02)
#define HMC5883L_REG_OUT_X_M          (0x03)
#define HMC5883L_REG_OUT_X_L          (0x04)
#define HMC5883L_REG_OUT_Z_M          (0x05)
#define HMC5883L_REG_OUT_Z_L          (0x06)
#define HMC5883L_REG_OUT_Y_M          (0x07)
#define HMC5883L_REG_OUT_Y_L          (0x08)
#define HMC5883L_REG_STATUS           (0x09)
#define HMC5883L_REG_IDENT_A          (0x0A)
#define HMC5883L_REG_IDENT_B          (0x0B)
#define HMC5883L_REG_IDENT_C          (0x0C)


// Once you have your heading, you must then add your 'Declination Angle',
// which is  the 'Error' of the magnetic field in your location.
//This project is -1°33' WEST (NEGATIVE) which is -1.55 degrees
//FORMULAR: (DEG + (MIN/60)) / (180/M_PI);
//TRANSFER to Radian ->  -0.0270526034 rad
#define  DECLINATION_ANGLE_RAD -0.02705
#define  DECLINATION_ANGLE_DEGREE -1.55
typedef struct
{
    float XAxis;
    float YAxis;
    float ZAxis;
    float XFinal;
    float YFinal;
    float ZFinal;

} HMC5883L;

void HMC5883L_Init(void);
void HMC5883L_WriteRegister(uint8_t regAddr, uint8_t data);
//void HMC5883L_ReadRegisters(uint8_t regAddr, uint8_t *buffer, uint16_t count);
int16_t HMC5883L_ReadRegisters(uint8_t regAddr);
void HMC5883L_Calibration(HMC5883L *hmc5883l);
void HMC5883L_ReadMagneticField(HMC5883L *hmc5883l);
double HMC5883L_GetHeading(HMC5883L *hmc5883l,char option);


#endif /* INC_HMC5883L_H_ */
