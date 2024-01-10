/*
 * moving.c
 *
 *  Created on: Nov 3, 2023
 *      Author: trand
 */
#include "moving.h"

#define LEFT_SPEED 24;
#define RIGHT_SPEED 29;

int leftSpeedValue = LEFT_SPEED;
int rightSpeedValue = RIGHT_SPEED;

static int maxSpeedRight = 29;
static int minSpeedRight = 15;

double deltaAngle;

float distance_live;
float travelledDis_live;
float counterClockwiseTurn_live;
float clockwiseTurn_live;


void STOP(){
	// Motor Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // (PA0 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // (PA1 - RPWM)
	// Motor Right (PB4 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // (PB4 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // (PB5 - RPWM)
}

void GO_FORWARD(){
	// Both tiến
	// Motor Left
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // (PA0 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, leftSpeedValue); // (PA1 - RPWM)
	// Motor Right (PB4 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // (PB4 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightSpeedValue); // (PB5 - RPWM)
}

void TURN_RIGHT(){
	// Motor Left đi tới
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // (PA0 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, leftSpeedValue); // (PA1 - RPWM)
	// Motor Right (PB4 - LPWM) đi lùi
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, rightSpeedValue); // (PB4 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // (PB5 - RPWM)
}

void TURN_LEFT(){
	// Motor Left lùi
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, leftSpeedValue); // (PA0 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // (PA1 - RPWM)
	// Motor Right (PB4 - LPWM) tiến
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // (PB4 - LPWM)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, rightSpeedValue); // (PB5 - RPWM)
}



// Sau khi quay đúng góc trực diện về phía điểm cần đến -> thực hiện đi thẳng tới điểm đó
// Yêu cầu phải send từ app về 4 coordinates của bounding box và grid list

void arrivePoint() {
	// PHẢI CẦN HÀM SAU KHI ĐỢI CURRENT GPS ổn định -> lưu startPoint lần đầu tiên (initializeStartPointGPS(currentGPS) đã làm)
	// targetAngle = round(convertRange360(azimuthAngle(startGPS, targetPos))); // get target angle value
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	deltaAngle = currentAngle - targetAngle;
	if(abs(deltaAngle) > 3) // Nếu chưa xoay qua đúng hướng -> thực hiện xoay đến khi đúng
		determineTurnDirection(currentAngle, targetAngle);
/*	else {
		// Sau khi đã xoay qua đúng -> bắt đầu tiến tới 1 khoảng distance
		float distance = Haversine_distancetan(startGPS, targetPos); // Nhưng grid distance là constant
		distance_live = distance; // watch live expression

		// Go straight distance
		float travelledDis = Haversine_distancetan(startGPS, currentGPS);
		travelledDis_live = travelledDis; // watch live expression
		//travelledDis = Haversine_distance(testStart, targetPos); // Đã đến

		if(abs(distance - travelledDis) <= 1) { // đã đến target position
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
			STOP(); // dừng lại để thực hiện chỉnh lại góc nếu lệch -> tiếp tục đi đến vị trí tiếp theo

			--------------------------------Chặng II-------------------------------------------

			startGPS = targetPos; // update vị trí hiện tại để tiếp tục tính khoảng cách -> đi tiếp
			gridIdx++; // cho robot biết điểm grid cần đến tiếp theo
		}
		else {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			driving();
			//GO_FORWARD(); // Thực hiện đi thẳng đến khi đạt 1 khoảng distance -> đã đến targetPosition
		}
	}*/
	else {
			// Sau khi đã xoay qua đúng -> bắt đầu tiến tới 1 khoảng distance
			/*
			float distance = Haversine_distancetan(startGPS, targetGPS); // Nhưng grid distance là constant
			distance_live = distance; // watch live expression

			// Go straight distance
			float travelledDis = Haversine_distancetan(startGPS, currentGPS);
			travelledDis_live = travelledDis; // watch live expression
			//travelledDis = Haversine_distance(testStart, targetGPS); // Đã đến
			 */
			float distance = Haversine_Distance(currentGPS, targetGPS); // Nhưng grid distance là constant
			distance_live = distance; // watch live expression
			if(distance <= 2) { // đã đến target position
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
				STOP(); // dừng lại để thực hiện chỉnh lại góc nếu lệch -> tiếp tục đi đến vị trí tiếp theo

				/*--------------------------------Chặng II-------------------------------------------*/

				startGPS = targetGPS; // update vị trí hiện tại để tiếp tục tính khoảng cách -> đi tiếp
				gridIdx++; // cho robot biết điểm grid cần đến tiếp theo
			}
			else {

				//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				// Thực hiện đi thẳng đến khi đạt 1 khoảng distance -> đã đến targetPosition
				driving();

			}
		}
}

/*-------Fuction to determine the best navigation to turn depend on Angle-------*/
void determineTurnDirection(float currentAngle, float targetAngle) {
    // Calculate angle to excute best turn
	// Clock Wise Turn from currentAngle to targetAngle
    float clockwiseTurn = fmod(targetAngle - currentAngle + 360.0, 360.0);
    // Counter Clock Wise Turn from currentAngle to targetAngle
    float counterClockwiseTurn = fmod(currentAngle - targetAngle + 360.0, 360.0);
    counterClockwiseTurn_live = counterClockwiseTurn;
    clockwiseTurn_live = clockwiseTurn;
    /*---------Calculation needed speed angle depend on angle change----------*/
    /*Khởi tạo biến giám sát tốc độ dưạ trên quay quanh trục Z
     *Tăng tốc độ quay của hệ thống khi nhận biết góc còn khoảng nhỏ
     */
    int targetGyroZ = 0;

    if(abs(deltaAngle)>10){
		targetGyroZ = 2* abs(deltaAngle);
	} else{
		targetGyroZ =20;
	}
	// Tính toán lỗi và lấy 50% giá trị lỗi để tăng tốc
	float error = targetGyroZ - abs(Gz);
	float adjustment = 0.5 * error;
    /*-------------------------------------------------------------------------*/

    if (clockwiseTurn < counterClockwiseTurn) {
    	rightSpeedValue = changeSpeed(rightSpeedValue, -1 + adjustment);
        TURN_RIGHT();
    } else {

        //Nếu cả hai góc quay đều bằng nhau, có thể rẽ theo bất kỳ hướng nào
        rightSpeedValue = changeSpeed(rightSpeedValue, +1 + adjustment);
    	TURN_LEFT();
    }
}

/*Hàm thực hiện chuyển động quay với chính nó*/
void rotate(){
	deltaAngle = currentAngle - targetAngle;
	if(abs(deltaAngle) <= 3){

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
		// Nếu đã hướng mặt về phía targetAngle tạm thời dừng để di chuyển thẳng (không quay nữa)
		STOP();

	} else {

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		// Tìm góc quay trái / phải thích hợp để hướng mặt về phía targetAngle
		determineTurnDirection(currentAngle, targetAngle);

	}
}

/*Hàm đẩy giá trị tốc độ cơ lên 1 bậc */
int changeSpeed(int motorSpeed, int increase){
	motorSpeed += increase;
	if(motorSpeed >maxSpeedRight){
		motorSpeed = maxSpeedRight;
	}else if(motorSpeed <minSpeedRight){
		motorSpeed = minSpeedRight ;
	}
	return motorSpeed;
}

/*Hàm điều chỉnh tốc độ hai bên động cơ*/
void controlSpeed(){
	deltaAngle = currentAngle - targetAngle;
	if(deltaAngle >=2){
		rightSpeedValue = changeSpeed(rightSpeedValue,round(-deltaAngle));
	}else if(deltaAngle <=-2){
		rightSpeedValue = changeSpeed(rightSpeedValue, abs(round(deltaAngle)));
	}else{
		leftSpeedValue = LEFT_SPEED;
		rightSpeedValue = RIGHT_SPEED;
	}
}

/*Hàm thực hiện di chuyển đi thẳng kèm theo điều kiện kiểm tra độ lệch*/
void driving(){
	deltaAngle = currentAngle - targetAngle;
	GO_FORWARD();
	if(deltaAngle != 0){
		//Điều khiển tốc độ động cơ để cân bằng
		controlSpeed();
		leftSpeedValue = LEFT_SPEED;
		GO_FORWARD();
	}

}




