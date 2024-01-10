/*
 * moving.h
 *
 *  Created on: Nov 4, 2023
 *      Author: trand
 */
#include "stm32f3xx_hal.h"


#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "stdbool.h"
#include "stdlib.h"
#include "geometric.h"
#include "math.h"


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;





extern bool isStart;
extern float currentAngle;
extern float targetAngle;
extern Coordinate startGPS;
extern Coordinate currentGPS;
extern Coordinate targetGPS;
extern float Gz;
extern int testNav;
extern int gridIdx;



void STOP();
void GO_FORWARD();
void TURN_LEFT();
void TURN_RIGHT();

void aim(float currentAngle, float targetAng);
void arrivePoint();

void rotate();
int changeSpeed(int motorSpeed, int increase);
void controlSpeed();
void driving();
void executeStartRoute(float currentDistance, float startDistance);
void determineTurnDirection(float currentAngle, float targetAngle);

#endif

