/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../lmic/lmic.h"
#include "../../stm32/debug.h"
#include "stdio.h"
/*------------OUT OF LORA-------------*/
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "MPU6050.h"
#include "HMC5883L.h"
#include "lwgps/lwgps.h"
#include "moving.h"
#include "geometric.h"
/*-------------------------*/
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Max size array
#define SIZEARRAY 50

/*---------------------MPU6050+HMC5883L----------------------*/
/*-------------Hệ thống cân bằng 9DOF - Trục tự do-----------*/
MPU6050 mpu6050;
MPU6050Error mpu6050error;
float currentAngle ; //Góc quay của hệ thống theo trục Z
float targetAngle; //Góc đích hệ thống cần đạt tới
HMC5883L hmc5883l;
float Gz;
bool firstTime = true;
/*-----------------------------------------------------------*/

/*-----------------------------GPS------------------------------*/
/* Số lượng mẫu dùng để làm trung bình dữ liệu GPS - Chống nhiễu*/
#define NUM_SAMPLES 20
lwgps_t gps;
float latitude_samples[NUM_SAMPLES];
float longitude_samples[NUM_SAMPLES];
int sample_index = 0;
//Khởi tạo điểm bắt đầu (�?iễm Tĩnh)
 Coordinate startGPS = {0.0f,0.0f};
// GPS after filter
Coordinate currentGPS = {0.0f,0.0f};
//GPS of shortest path in Markers from startGPS
Coordinate targetGPS = {16.075417285156828, 108.15257015675355};
//Coordinate targetGPS = {16.075421557735048, 108.15258639483889};

uint8_t isStartSaved = 0;
/*--------------------------------------------------------------*/


/*Nhận dữ liệu Markers từ Server -> STM32 thông qua LoRa
 * Mảng lưu khoảng cách để tìm ngắn nhất*/
static Coordinate markers[2] = {
		{16.075194111747805, 108.15276546075143},
		{16.0711222, 108.1523200},

};


Coordinate gridList[10] = {
		{16.075425130506527, 108.15259298609973},
		{16.075428518189263, 108.15261629679365}
};

int gridIdx = 0; // Khi chưa đến marker 0
int CCRleft1, CCRleft2, CCRright1, CCRright2;
//Distance from Start Point to Marker 0
float firstDistance= 0.0;
//Distance from
float currentDistance = 0.0;

int32_t lat_int ;
int32_t lat_fractional;
/*------------------------------------------------------*/

/*----For SPI1----*/
uint8_t RxBuffer;
char *tmp;
int dataRx = 0;
uint8_t receivedData;
/*----------------*/

/*----For USART 1----*/
uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;
/*-------------------*/

/*---------For ADC---------*/
float battery_voltage =0.0f;
float speed =0.0f;
uint32_t ADC_value;
/*-------------------------*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int a =0;
int fPort ;
uint8_t result;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// application router ID (LSBF)  < ------- IMPORTANT
// application router ID (LSBF)
static const uint8_t APPEUI[8]  = {}; //{ 0xC1 , 0x67 , 0x01 , 0xD0 , 0x7E , 0xD5 , 0xB3 , 0x70};

// unique device ID (LSBF)
static const uint8_t DEVEUI[8]  ={ 0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x21, 0x35 }; //{ 0x5b , 0xfe , 0xA1 , 0x59 , 0x2f , 0xf1 , 0x56 , 0x00 };

// device-specific AES key (derived from device EUI)
static const uint8_t DEVKEY[16] = {}; //{ 0x02, 0x25, 0xC3, 0x06, 0x01, 0xAF, 0xA9, 0xEC, 0x05, 0x74, 0xA2, 0x66, 0xA5, 0xAA, 0xE3, 0x18 };
// network.
static const uint8_t NWKSKEY[16] = { 0x5F, 0xCC, 0x77, 0xA8, 0x6A, 0xCB, 0x59, 0x7D, 0x5B, 0xD9, 0x80, 0x23, 0x13, 0x45, 0x94, 0x73 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const uint8_t APPSKEY[16] = { 0xAC, 0x06, 0x47, 0xBB, 0x19, 0xE8, 0x25, 0x1D, 0xF3, 0xB5, 0x70, 0xC6, 0x0C, 0xB0, 0x9A, 0x54 };

// LoRaWAN end-device address (DevAddr)
static const uint32_t DEVADDR = 0x27FD1B80; // <-- Change this address for every node!

//   ABP
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) {
		if(rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
		} else {
			lwgps_process(&gps, rx_buffer, rx_index+1);
			rx_index = 0;
			rx_data = 0;
		}
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}

void deleteBuffer(char* data)
{
  uint8_t len = strlen(data);
  for(uint8_t i = 0; i < len; i++)
  {
		data[i] = 0;
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void os_getArtEui (uint8_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (uint8_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (uint8_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

#define TX_INTERVAL 10

#define TX_TIMEOUT 60
static uint8_t data[24];
static osjob_t sendjob;

void do_send(osjob_t* j){
	ostime_t tm = os_getTime();
		printf("Call %s %lums\n", __func__, (unsigned long)osticks2ms(tm));

		if (!(LMIC.opmode & OP_TXRXPEND)) {

			double lat_float = currentGPS.latitude * 1000000.0;
			double lon_float = currentGPS.longitude * 1000000.0;

			 lat_int = (int32_t)lat_float; // Integer part of latitude
			 int32_t lon_int = (int32_t)lon_float; // Integer part of longitude

			// Calculate fractional parts for latitude and longitude with 10 decimal places
			double lat_fractional_double = (lat_float - lat_int)*1000.0;
			double lon_fractional_double = (lon_float - lon_int) * 1000.0;

			// Round the fractional parts to integers
			 lat_fractional = (int32_t)(lat_fractional_double ); // Round to the nearest integer
			int32_t lon_fractional = (int32_t)(lon_fractional_double ); // Round to the nearest integer

			//Convert speed and battery percentage to integers for transmission
			int32_t speedInt = (int32_t)(speed * 100); // Multiply by 100 to keep two decimal places
			int32_t batteryPercentageInt = (int32_t)(battery_voltage * 100); // Multiply by 100 to keep two decimal places

			// Now convert integer and fractional parts into bytes
			data[0] = (lat_int >> 24) & 0xFF;
			data[1] = (lat_int >> 16) & 0xFF;
			data[2] = (lat_int >> 8) & 0xFF;
			data[3] = lat_int & 0xFF;

			data[4] = (lat_fractional >> 24) & 0xFF;
			data[5] = (lat_fractional >> 16) & 0xFF;
			data[6] = (lat_fractional >> 8) & 0xFF;
			data[7] = lat_fractional & 0xFF;

			data[8] = (lon_int >> 24) & 0xFF;
			data[9] = (lon_int >> 16) & 0xFF;
			data[10] = (lon_int >> 8) & 0xFF;
			data[11] = lon_int & 0xFF;

			data[12] = (lon_fractional >> 24) & 0xFF;
			data[13] = (lon_fractional >> 16) & 0xFF;
			data[14] = (lon_fractional >> 8) & 0xFF;
			data[15] = lon_fractional & 0xFF;

			data[16] = (speedInt >> 24) & 0xFF;
			data[17] = (speedInt >> 16) & 0xFF;
			data[18] = (speedInt >> 8) & 0xFF;
			data[19] = speedInt & 0xFF;

			data[20] = (batteryPercentageInt >> 24) & 0xFF;
			data[21] = (batteryPercentageInt >> 16) & 0xFF;
			data[22] = (batteryPercentageInt >> 8) & 0xFF;
			data[23] = batteryPercentageInt & 0xFF;

			LMIC_setTxData2(1, data, sizeof(data), 0);
			for (int i = 0; i<=sizeof(data); i++) data[i] = 0;
		}
		os_setTimedCallback(j, tm+sec2osticks(TX_TIMEOUT), do_send);
}
void onEvent (ev_t ev) {
    debug_event(ev);
    debug_str("\n");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            debug_str("EV_SCAN_TIMEOUT\n");
            break;
        case EV_BEACON_FOUND:
            debug_str("EV_BEACON_FOUND\n");
            break;
        case EV_BEACON_MISSED:
            debug_str("EV_BEACON_MISSED\n");
            break;
        case EV_BEACON_TRACKED:
            debug_str("EV_BEACON_TRACKED\n");
            break;
        case EV_JOINING:
            debug_str("EV_JOINING\n");
            break;
        case EV_JOINED:
            debug_str("EV_JOINED\n");
            break;
        case EV_RFU1:
            debug_str("EV_RFU1\n");
            break;
        case EV_JOIN_FAILED:
            debug_str("EV_JOIN_FAILED\n");
            break;
        case EV_REJOIN_FAILED:
            debug_str("EV_REJOIN_FAILED\n");
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_TXCOMPLETE:
            debug_str("EV_TXCOMPLETE (includes waiting for RX windows)\n");
            if (LMIC.txrxFlags & TXRX_ACK)
              debug_str("Received ack\n");
            if (LMIC.dataLen) {
            	a=1;
              debug_str("Received \n");
              debug_int(LMIC.dataLen);
              debug_str(" bytes of payload\n");
            }

            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            debug_str("EV_LOST_TSYNC\n");
            break;
        case EV_RESET:
            debug_str("EV_RESET\n");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            debug_str("EV_RXCOMPLETE\n");
            if(LMIC.dataLen >0){
          	  result = LMIC.frame[LMIC.dataBeg + 0];
            if (result == 1)  {
          	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
           }
           if (result == 2)  {
          	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
           }
           }
            break;
        case EV_LINK_DEAD:
            debug_str("EV_LINK_DEAD\n");
            break;
        case EV_LINK_ALIVE:
            debug_str("EV_LINK_ALIVE\n");
            break;
         default:
            debug_str("Unknown event\n");
            break;
    }
}

void initfunc(osjob_t* j)
{
	   // Reset the MAC state. Session and pending data transfers will be discarded.
	   LMIC_reset();
	   LMIC_startJoining();
	   // Set static session parameters. Instead of dynamically establishing a session
	   // by joining the network, precomputed session parameters are be provided.
	   // On AVR, these values are stored in flash and only copied to RAM
	   // once. Copy them to a temporary buffer here, LMIC_setSession will
	   // copy them into a buffer of its own again.
	    uint8_t appskey[sizeof(APPSKEY)];
	    uint8_t nwkskey[sizeof(NWKSKEY)];
	    memcpy(appskey, APPSKEY, sizeof(APPSKEY));
	    memcpy(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);


	  #if defined(CFG_eu868)
	   // Set up the channels used by the Things Network, which corresponds
	   // to the defaults of most gateways. Without this, only three base
	   // channels from the LoRaWAN specification are used, which certainly
	   // works, so it is good for debugging, but can overload those
	   // frequencies, so be sure to configure the full frequency range of
	   // your network here (unless your network autoconfigures them).
	   // Setting up channels should happen after LMIC_setSession, as that
	   // configures the minimal channel set.
	   // NA-US channels 0-71 are configured automatically
	   LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	   LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
	   // TTN defines an additional channel at 869.525Mhz using SF9 for class B
	   // devices' ping slots. LMIC does not have an easy way to define set this
	   // frequency and support for class B is spotty and untested, so this
	   // frequency is not configured here.
	   #elif defined(CFG_us915)
	   // NA-US channels 0-71 are configured automatically
	   // but only one group of 8 should (a subband) should be active
	   // TTN recommends the second sub band, 1 in a zero based count.
	   // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
	   LMIC_selectSubBand(1);
	  #endif

	   // Disable link check validation
	   LMIC_setLinkCheckMode(0);

	   // TTN uses SF9 for its RX2 window.
	   LMIC.dn2Dr = DR_SF9;

	   // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	   LMIC_setDrTxpow(DR_SF9,14);
	   os_setTimedCallback(&sendjob, os_getTime(), do_send);
	   // Start job

	  // time= HAL_GetTick();
}



// Hàm cập nhật giá trị GPS bằng cách làm trung bình các mẫu
void UpdateGPS(double new_latitude, double new_longitude) {

    // Lưu giá trị mới vào mảng mẫu
    latitude_samples[sample_index] = new_latitude;
    longitude_samples[sample_index] = new_longitude;

    // Tăng chỉ số mẫu
    sample_index = (sample_index + 1) % NUM_SAMPLES;

    // Tính giá trị trung bình từ các mẫu
    double sum_latitude = 0.0f;
    double sum_longitude = 0.0f;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum_latitude += latitude_samples[i];
        sum_longitude += longitude_samples[i];
    }

    // Cập nhật giá trị GPS bằng giá trị trung bình
    currentGPS.latitude = sum_latitude / NUM_SAMPLES;
    currentGPS.longitude = sum_longitude / NUM_SAMPLES;

}



/*Function save Start GPS at the first time TURN ON SYSTEM*/
void initializeStartPointGPS(Coordinate current){
	if (current.latitude != 0 && isStartSaved <= 250) {
		startGPS = current;
		isStartSaved++;
	}
}

/*Function save the first target angle for first time open sytem with no data markers*/
void initializeFirstAngle(){
	if ( firstTime ) {
		targetAngle = currentAngle;
		firstTime = false;
	}
}


/*Function calculation angle of system depend on X-Y-Z Axis*/
void calculateAngleOfSystem(){
	HMC5883L_ReadMagneticField(&hmc5883l);
    MPU6050_Read_All(&mpu6050, &mpu6050error,&hmc5883l);
    Gz = mpu6050.Gz;
    //Current angle of robot
    currentAngle = round(mpu6050.Angle[2]);
    HAL_Delay(1);

}


/* Function calculating from the startPoint to 1 of Grid[0] */
float informationFirstPhase(){
	targetGPS.longitude = markers[0].longitude;
	targetGPS.latitude = markers[0].latitude;
	return Haversine_Distance(startGPS, targetGPS);;

}


/*Function to find target angle (Angle of shorsted path)*/
void findAngleForFirstPhase(){

	targetAngle = round(convertRange360(Azimuth_Angle(startGPS, targetGPS)));
}

/* Function to calculate battery percentage*/
float calculateBatteryPercentage(uint16_t adcValue) {
    uint16_t ADC_min = 2444*(3.37/6.7);
    uint16_t ADC_max = 4095;


    float batteryPercentage = ((float)(adcValue - ADC_min) / (float)(ADC_max - ADC_min)) * 100.0f;

    if (batteryPercentage > 100.0f) {
        batteryPercentage = 100.0f;
    } else if (batteryPercentage < 0.0f) {
        batteryPercentage = 0.0f;
    }

    return batteryPercentage;
}

/* Function to read ADC value for Battery Percentage*/
void processingBatteryPercentage(){
	HAL_ADC_Start_DMA(&hadc1, &ADC_value, 1);
	speed = lwgps_to_speed(gps.speed, lwgps_speed_mps);
	battery_voltage = calculateBatteryPercentage(ADC_value);
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  /*------------PWM Motor Init-----------*/
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /*------------------------------ -------*/


  /*-----------Robot GPS Init-------------*/
  lwgps_init(&gps);
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /*--------------------------------------*/

  /*---------------9DOF Init------------------*/
  MPU6050_Init();
  MPU6050_Calibration(&mpu6050, &mpu6050error);
  HAL_Delay(20);
  HMC5883L_Init();
  HMC5883L_Calibration(&hmc5883l);
  /*------------------------------------------*/

  /*------Raspberry Pi 4 interract-------*/
   // HAL_SPI_Receive_DMA(&hspi1, &RxBuffer, 1);
   /*-------------------------------------*/


  /*----------LORA Init------------*/
  HAL_TIM_Base_Start_IT(&htim4);    // <-----------  change to your setup
  HAL_TIM_Base_Start_IT(&htim7);    // <-----------  change to your setup
  __HAL_SPI_ENABLE(&hspi2);      	// <-----------  change to your setup
  /*------------------------------*/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	osjob_t initjob;
	os_init();
	initfunc(&initjob);
	do_send(&sendjob);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*-----Basic Update Position and Angle of system-----*/
	   UpdateGPS(gps.latitude, gps.longitude);
	   calculateAngleOfSystem();
	   processingBatteryPercentage();
	  /*---------------------------------------------------*/


	   if(sizeof(markers) != 0){
		   if(RxBuffer == 'a'){
			   TURN_LEFT();
		   	   		tmp = "left";
		   }	else if(RxBuffer == 'b'){
			   TURN_RIGHT();
		   	   		tmp = "right";
		   } 	else if(RxBuffer == 'c'){
			   	   driving();
					tmp = "straight";
		   }else {
			   STOP();
			   initializeStartPointGPS(currentGPS);
			   	  	 HAL_Delay(10);
			   	  	 /*-----------------------Chặng - I-----------------------*/
			   	  //	 targetGPS = (gridIdx < 0) ? markers[0] : gridList[gridIdx];
			   	  	 targetGPS =gridList[gridIdx];
			   	  	 firstDistance = Haversine_Distance(startGPS, targetGPS);
			   	  	 findAngleForFirstPhase();
			   	  	 arrivePoint(targetGPS);
		   }
	  }else {

		  //When no data in markers , system will rotate for itself to avoid water current
		  initializeFirstAngle();
		  rotate();
	  }


	  /*----LoRa send data---*/
	  os_runloop_once();
	  /*---------------------*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 244-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 30000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1221-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1249;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 63999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|lora_NSS_PIN_Pin|lora_Reset_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 lora_NSS_PIN_Pin lora_Reset_PIN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13|lora_NSS_PIN_Pin|lora_Reset_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : lora_DIO1_PIN_Pin lora_DIO2_PIN_Pin lora_DIO0_PIN_Pin */
  GPIO_InitStruct.Pin = lora_DIO1_PIN_Pin|lora_DIO2_PIN_Pin|lora_DIO0_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
